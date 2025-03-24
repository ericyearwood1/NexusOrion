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

using Meta.XR.ImmersiveDebugger.Manager;
using Meta.XR.ImmersiveDebugger.Utils;
using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using UnityEngine.UI;

namespace Meta.XR.ImmersiveDebugger.UserInterface
{
    internal interface IDebugUIPanel
    {
        public IInspector RegisterInspector(InstanceHandle instance);
        public void UnregisterInspector(InstanceHandle instance);
        public IInspector GetInspector(InstanceHandle instance);
    }

    internal interface IInspector
    {
        public IMember RegisterMember(MemberInfo memberInfo);
        public IMember GetMember(MemberInfo memberInfo);
    }

    internal interface IMember
    {
        public void RegisterAttribute(DebugMember attribute);
        public void RegisterAction(Action onClick);
        public void RegisterGizmo(Action<bool> onStateChanged);
        public void RegisterWatch(Watch watch);
        public void RegisterTweak(Tweak tweak);

#if OVR_INTERNAL_CODE
#if OVRPLUGIN_TESTING
        public Action GetAction();
        public Action<bool> GetGizmoToggle();
        public DebugMember GetAttribute();
        public Watch GetWatch();
        public Tweak GetTweak();
#endif // OVRPLUGIN_TESTING
#endif // OVR_INTERNAL_CODE
    }

#if OVR_INTERNAL_CODE
#if OVRPLUGIN_TESTING
    internal class MockMember : IMember
    {
        private Action _action;
        private Action<bool> _gizmoToggle;
        private DebugMember _attribute;
        private Watch _watch;
        private Tweak _tweak;

        public void RegisterAttribute(DebugMember attribute) { _attribute = attribute; }
        public void RegisterAction(Action onClick) { _action = onClick; }
        public void RegisterGizmo(Action<bool> onStateChanged) { _gizmoToggle = onStateChanged; }
        public void RegisterWatch(Watch watch) { _watch = watch; }
        public void RegisterTweak(Tweak tweak) { _tweak = tweak;}

        public Action GetAction() => _action;
        public Action<bool> GetGizmoToggle() => _gizmoToggle;
        public DebugMember GetAttribute() => _attribute;
        public Watch GetWatch() => _watch;
        public Tweak GetTweak() => _tweak;
    }

    internal class MockInspector : IInspector
    {
        private readonly Dictionary<MemberInfo, MockMember> _members = new Dictionary<MemberInfo, MockMember>();

        public IMember RegisterMember(MemberInfo memberInfo)
        {
            if (!_members.TryGetValue(memberInfo, out var member))
            {
                member = new MockMember();
                _members.Add(memberInfo, member);
            }

            return member;
        }

        public IMember GetMember(MemberInfo memberInfo)
        {
            _members.TryGetValue(memberInfo, out var member);
            return member;
        }
    }

    internal class MockUIPanel : MonoBehaviour, IDebugUIPanel
    {
        private readonly Dictionary<Type, Dictionary<InstanceHandle, MockInspector>> _inspectors =
            new Dictionary<Type, Dictionary<InstanceHandle, MockInspector>>();

        public IInspector GetInspector(InstanceHandle instanceHandle)
        {
            MockInspector inspector = null;
            _inspectors.TryGetValue(instanceHandle.Type, out var registry);
            registry?.TryGetValue(instanceHandle, out inspector);
            return inspector;
        }

        public IInspector RegisterInspector(InstanceHandle instanceHandle)
        {
            if (!_inspectors.TryGetValue(instanceHandle.Type, out var registry))
            {
                registry = new Dictionary<InstanceHandle, MockInspector>();
                _inspectors.Add(instanceHandle.Type, registry);
            }

            if (!registry.TryGetValue(instanceHandle, out var inspector))
            {
                inspector = new MockInspector();
                registry.Add(instanceHandle, inspector);
            }

            return inspector;
        }

        public void UnregisterInspector(InstanceHandle instanceHandle)
        {
            _inspectors.TryGetValue(instanceHandle.Type, out var registry);
            registry?.Remove(instanceHandle);
        }
    }
#endif // OVRPLUGIN_TESTING
#endif // OVR_INTERNAL_CODE
}

#endif // OVR_INTERNAL_CODE
