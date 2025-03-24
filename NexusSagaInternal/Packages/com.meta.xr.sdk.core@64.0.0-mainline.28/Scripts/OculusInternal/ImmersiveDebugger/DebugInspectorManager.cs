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

#if OVR_INTERNAL_CODE // IMMERSIVE DEBUGGER IS INTERNAL

using System;
using System.Collections.Generic;
using System.Reflection;
using Meta.XR.ImmersiveDebugger.Gizmo;
using Meta.XR.ImmersiveDebugger.Manager;
using Meta.XR.ImmersiveDebugger.UserInterface;
using Meta.XR.ImmersiveDebugger.Utils;
using UnityEngine;

namespace Meta.XR.ImmersiveDebugger
{
    internal class DebugInspectorManager
    {
        private abstract class ManagerFromInspector : IDebugManager
        {
            private readonly Dictionary<Type, List<MemberInfo>> _dictionary = new Dictionary<Type, List<MemberInfo>>();
            private IDebugUIPanel _uiPanel;
            protected InstanceCache _instanceCache;

            public void Setup(IDebugUIPanel panel, InstanceCache cache)
            {
                _uiPanel = panel;
                _instanceCache = cache;
            }

            public void ProcessType(Type type) => throw new NotImplementedException();

            public void ProcessTypeFromInspector(Type type, InstanceHandle handle, MemberInfo memberInfo, DebugMember memberAttribute)
            {
                var uiInspector = _uiPanel.RegisterInspector(handle);
                var member = uiInspector.RegisterMember(memberInfo);
                member.RegisterAttribute(memberAttribute);

                if (RegisterSpecialisedWidget(member, memberInfo, memberAttribute, handle))
                {
                    if (!_dictionary.TryGetValue(type, out var list))
                    {
                        list = new List<MemberInfo>();
                        _dictionary.Add(type, list);
                    }

                    if (!list.Contains(memberInfo))
                    {
                        list.Add(memberInfo);
                    }
                }
            }
            public int GetCountPerType(Type type)
            {
                _dictionary.TryGetValue(type, out var actions);
                return actions?.Count ?? 0;
            }

            protected abstract bool RegisterSpecialisedWidget(IMember member, MemberInfo memberInfo, DebugMember memberAttribute, InstanceHandle handle);
            public abstract string TelemetryAnnotation { get; }
        }

        private class TweakManagerFromInspector : ManagerFromInspector
        {
            protected override bool RegisterSpecialisedWidget(IMember member, MemberInfo memberInfo,
                DebugMember memberAttribute, InstanceHandle handle)
            {
                if (!memberAttribute.Tweakable) return false;

                member.RegisterTweak(TweakUtils.Create(memberInfo, memberAttribute, handle.Instance));
                return true;
            }

            public override string TelemetryAnnotation => Telemetry.AnnotationType.Tweaks;
        }

        private class ActionManagerFromInspector : ManagerFromInspector
        {
            protected override bool RegisterSpecialisedWidget(IMember member, MemberInfo memberInfo,
                DebugMember memberAttribute, InstanceHandle handle)
            {
                if (memberInfo.MemberType == MemberTypes.Method)
                {
                    var method = memberInfo as MethodInfo;
                    if (method != null && method.GetParameters().Length == 0 && method.ReturnType == typeof(void))
                    {
                        member.RegisterAction(() => method.Invoke(handle.Instance, null));
                        return true;
                    }
                }

                return false;
            }

            public override string TelemetryAnnotation => Telemetry.AnnotationType.Actions;
        }

        private class WatchManagerFromInspector : ManagerFromInspector
        {
            protected override bool RegisterSpecialisedWidget(IMember member, MemberInfo memberInfo,
                DebugMember memberAttribute, InstanceHandle handle)
            {
                if (memberInfo.MemberType == MemberTypes.Property | memberInfo.MemberType == MemberTypes.Field)
                {
                    member.RegisterWatch(WatchUtils.Create(memberInfo, handle.Instance));
                    return true;
                }

                return false;
            }

            public override string TelemetryAnnotation => Telemetry.AnnotationType.Watches;
        }

        private class GizmoManagerFromInspector : ManagerFromInspector
        {
            private readonly Dictionary<MemberInfo, GizmoRendererManager> _memberToGizmoRendererManagerDict = new();
            protected override bool RegisterSpecialisedWidget(IMember member, MemberInfo memberInfo,
                DebugMember memberAttribute, InstanceHandle handle)
            {
                if (memberAttribute.GizmoType == DebugGizmoType.None)
                {
                    return false;
                }

                if (!_memberToGizmoRendererManagerDict.TryGetValue(memberInfo, out var gizmoRendererManager))
                {
                    if (AddGizmo(handle.Type, memberInfo, memberAttribute, out gizmoRendererManager))
                    {
                        _memberToGizmoRendererManagerDict[memberInfo] = gizmoRendererManager;
                    }
                }

                if (gizmoRendererManager != null)
                {
                    member.RegisterGizmo((state) => _memberToGizmoRendererManagerDict[memberInfo].SetState(handle.Instance, state));
                    return true;
                }
                return false;
            }

            private bool AddGizmo(Type type, MemberInfo member, DebugMember gizmoAttribute, out GizmoRendererManager gizmoRendererManager)
            {
                if (!GizmoTypesRegistry.IsValidDataTypeForGizmoType(member.GetDataType(), gizmoAttribute.GizmoType))
                {
                    Debug.LogWarning($"Invalid registration of gizmo {member.Name}: type not matching gizmo type");
                    gizmoRendererManager = null;
                    return false;
                }

                var gizmo = new GameObject($"{member.Name}Gizmo");
                gizmoRendererManager = gizmo.AddComponent<GizmoRendererManager>();
                gizmoRendererManager.Setup(type, member, gizmoAttribute.GizmoType, gizmoAttribute.Color, _instanceCache);
                return true;
            }

            public override string TelemetryAnnotation => Telemetry.AnnotationType.Gizmos;
        }

        private static DebugInspectorManager _instance;
        public static DebugInspectorManager Instance => _instance ??= new DebugInspectorManager();

        private readonly InstanceCache _instanceCache = new InstanceCache();
        private readonly List<IDebugManager> _subDebugManagers = new List<IDebugManager>();
        private readonly List<DebugInspector> _inspectors = new List<DebugInspector>();
        private static IDebugUIPanel _uiPanel;

        private DebugInspectorManager()
        {
            if (DebugManager.Instance == null)
            {
                DebugManager.OnReady += OnReady;
            }
            else
            {
                OnReady(DebugManager.Instance);
            }
        }

        internal static void Destroy()
        {
            if (_instance != null)
            {
                DebugManager.OnReady -= _instance.OnReady;
            }
        }

        private void InitSubManagers()
        {
            RegisterManager<GizmoManagerFromInspector>();
            RegisterManager<WatchManagerFromInspector>();
            RegisterManager<ActionManagerFromInspector>();
            RegisterManager<TweakManagerFromInspector>();
        }

        private void RegisterManager<TManagerType>()
            where TManagerType : IDebugManager, new()
        {
            var manager = new TManagerType();
            manager.Setup(_uiPanel, _instanceCache);
            _subDebugManagers.Add(manager);
        }

        public void RegisterInspector(DebugInspector inspector)
        {
            _inspectors.Add(inspector);
            ProcessInspector(inspector);
        }

        public void UnregisterInspector(DebugInspector inspector)
        {
            UnprocessInspector(inspector);
            _inspectors.Remove(inspector);
        }

        private void OnReady(DebugManager debugManager)
        {
            _uiPanel = debugManager.UiPanel;
            Telemetry.TelemetryTracker.Start(Telemetry.Method.DebugInspector, _subDebugManagers, _instanceCache, debugManager);

            InitSubManagers();

            foreach (var inspector in _inspectors)
            {
                ProcessInspector(inspector);
            }
        }

        private void ProcessInspector(DebugInspector inspector)
        {
            if (_uiPanel == null) return;

            foreach (var entry in inspector.Registry.Handles)
            {
                if (!entry.Visible) continue;

                var handle = entry.InstanceHandle;
                _instanceCache.RegisterHandle(handle);
                var uiInspector = _uiPanel.RegisterInspector(handle);
                foreach (var memberEntry in entry.inspectedMembers)
                {
                    if (!memberEntry.Visible) continue;

                    var memberInfo = memberEntry.MemberInfo;
                    if (memberInfo == null) continue;

                    var attribute = memberEntry.attribute;
                    if (attribute == null) continue;

                    foreach (var manager in _subDebugManagers)
                    {
                        manager.ProcessTypeFromInspector(handle.Type, handle, memberInfo, attribute);
                    }
                }
            }
        }

        private void UnprocessInspector(DebugInspector inspector)
        {
            if (_uiPanel == null) return;

            foreach (var entry in inspector.Registry.Handles)
            {
                var handle = entry.InstanceHandle;
                _uiPanel.UnregisterInspector(handle);
                _instanceCache.UnregisterHandle(handle);
            }
        }
    }
}

#endif // OVR_INTERNAL_CODE // IMMERSIVE DEBUGGER IS INTERNAL
