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

using Meta.XR.ImmersiveDebugger.UserInterface.Generic;
using Meta.XR.ImmersiveDebugger.Utils;
using System;
using System.Collections.Generic;

namespace Meta.XR.ImmersiveDebugger.UserInterface
{
    internal class InspectorPanel : DebugPanel, IDebugUIPanel
    {
        private ScrollView _scrollView;
        private Flex Flex => _scrollView.Flex;

        private readonly Dictionary<Type, Dictionary<InstanceHandle, Inspector>> _registries =
            new Dictionary<Type, Dictionary<InstanceHandle, Inspector>>();

        protected override void Setup(Controller owner)
        {
            base.Setup(owner);

            _scrollView = Append<ScrollView>("main");
            _scrollView.LayoutStyle = Style.Load<LayoutStyle>("PanelScrollView");
            Flex.LayoutStyle = Style.Load<LayoutStyle>("InspectorMainFlex");
        }

        public IInspector RegisterInspector(InstanceHandle instanceHandle)
        {
            if (!_registries.TryGetValue(instanceHandle.Type, out var registry))
            {
                registry = new Dictionary<InstanceHandle, Inspector>();
                _registries.Add(instanceHandle.Type, registry);
            }

            if (!registry.TryGetValue(instanceHandle, out var inspector))
            {
                var previousScroll = _scrollView.ScrollRect.verticalNormalizedPosition;

                var instance = instanceHandle.Instance;
                var inspectorName = instance != null ? instance.name : instanceHandle.Type.Name;
                inspector = Flex.Append<Inspector>(inspectorName);
                inspector.LayoutStyle = Style.Load<LayoutStyle>("Inspector");
                var inspectorTitle = instance != null ? $"{instance.name} - {instanceHandle.Type.Name}" : $"{instanceHandle.Type.Name}";
                inspector.Title = inspectorTitle;
                registry.Add(instanceHandle, inspector);

                Flex.RefreshLayout();

                _scrollView.ScrollRect.verticalNormalizedPosition = previousScroll;
            }

            return inspector;
        }

        public void UnregisterInspector(InstanceHandle instanceHandle)
        {
            Inspector inspector = null;
            _registries.TryGetValue(instanceHandle.Type, out var registry);
            registry?.TryGetValue(instanceHandle, out inspector);
            if (inspector != null)
            {
                var previousScroll = _scrollView.ScrollRect.verticalNormalizedPosition;

                // Unregister the inspector
                registry?.Remove(instanceHandle);

                // Destroy the inspector
                Flex.Remove(inspector, true);

                _scrollView.ScrollRect.verticalNormalizedPosition = previousScroll;
            }
        }

        public IInspector GetInspector(InstanceHandle instanceHandle)
        {
            Inspector inspector = null;
            _registries.TryGetValue(instanceHandle.Type, out var registry);
            registry?.TryGetValue(instanceHandle, out inspector);
            return inspector;
        }
    }
}

#endif // OVR_INTERNAL_CODE
