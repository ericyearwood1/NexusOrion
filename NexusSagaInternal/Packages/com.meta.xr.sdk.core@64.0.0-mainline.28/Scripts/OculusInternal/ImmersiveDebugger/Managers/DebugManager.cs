/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if OVR_INTERNAL_CODE

using Meta.XR.ImmersiveDebugger.UserInterface;
using Meta.XR.ImmersiveDebugger.Utils;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Meta.XR.ImmersiveDebugger.Manager
{
    /// Retrieve debugging requests, update runtime values
    internal class DebugManager : MonoBehaviour
    {
        public static DebugManager Instance { get; private set; }

        public static event Action<DebugManager> OnReady;
        public event Action OnStartAction;
        public event Action OnFocusLostAction;
        public event Action OnDisableAction;
        public event Action OnUpdateAction;

        private IDebugUIPanel _uiPanel;
        protected readonly InstanceCache _instanceCache = new InstanceCache();
        protected List<IDebugManager> _subDebugManagers = new List<IDebugManager>();

        internal bool ShouldRetrieveInstances = false;
        private const float RetrievalIntervalInSec = 1.0f;
        private float _lastRetrievedTime;

        public IDebugUIPanel UiPanel => _uiPanel;

        private void Awake()
        {
            Instance = this;
            _instanceCache.OnCacheChangedForTypeEvent += ProcessLoadedTypeBySubManagers;
            _instanceCache.OnInstanceAdded += RegisterInspector;
            _instanceCache.OnInstanceRemoved += UnregisterInspector;
        }

        private void Start()
        {
            _uiPanel = GetComponentInChildren<IDebugUIPanel>(true);
            InitSubManagers();

            Telemetry.TelemetryTracker.Start(Telemetry.Method.Attributes, _subDebugManagers, _instanceCache, this);

            AssemblyParser.RegisterAssemblyTypes(_instanceCache.RegisterClassTypes);
            ShouldRetrieveInstances = true;
            RetrieveInstancesIfNeeded();

            OnReady?.Invoke(this);
            OnStartAction?.Invoke();
        }

        void OnApplicationFocus(bool hasFocus)
        {
            if (!hasFocus)
            {
                OnFocusLostAction?.Invoke();
            }
        }

        void OnDisable()
        {
            OnDisableAction?.Invoke();
        }

        private void OnDestroy()
        {
            AssemblyParser.Unregister(_instanceCache.RegisterClassTypes);
        }

        private void Update()
        {
            RetrieveInstancesIfNeeded();

            OnUpdateAction?.Invoke();
        }

        private void RetrieveInstancesIfNeeded()
        {
            if (Time.time - _lastRetrievedTime > RetrievalIntervalInSec)
            {
                ShouldRetrieveInstances = true;
                _lastRetrievedTime = Time.time;
            }

            if (ShouldRetrieveInstances)
            {
                _instanceCache.RetrieveInstances();
                ShouldRetrieveInstances = false;
            }
        }

        protected virtual void InitSubManagers()
        {
            RegisterManager<GizmoManager>();
            RegisterManager<WatchManager>();
            RegisterManager<ActionManager>();
            RegisterManager<TweakManager>();
        }

        private void RegisterManager<TManagerType>()
            where TManagerType : IDebugManager, new()
        {
            var manager = new TManagerType();
            manager.Setup(_uiPanel, _instanceCache);
            _subDebugManagers.Add(manager);
        }

        private void ProcessLoadedTypeBySubManagers(Type type)
        {
            foreach (var manager in _subDebugManagers)
            {
                manager.ProcessType(type);
            }
        }

        public IInspector RegisterInspector(InstanceHandle handle)
        {
            return _uiPanel.RegisterInspector(handle);
        }

        public void UnregisterInspector(InstanceHandle handle)
        {
            _uiPanel.UnregisterInspector(handle);
        }
    }
}

#endif // OVR_INTERNAL_CODE
