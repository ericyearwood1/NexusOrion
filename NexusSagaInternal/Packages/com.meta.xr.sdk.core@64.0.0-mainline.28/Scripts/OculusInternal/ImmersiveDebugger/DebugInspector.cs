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
using Meta.XR.ImmersiveDebugger.Utils;
using UnityEngine;

namespace Meta.XR.ImmersiveDebugger
{
    [ExecuteInEditMode]
    public class DebugInspector : MonoBehaviour
    {
        [Serializable]
        internal class InspectionRegistry
        {
            [SerializeField] private List<InspectedHandle> handles = new List<InspectedHandle>();

            internal List<InspectedHandle> Handles => handles;

            public void Initialize(DebugInspector owner)
            {
                // Initialize Handles
                foreach (var handle in handles)
                {
                    handle.Initialize(owner);
                }

                // Search for new components
                var components = owner.GetComponents<Component>();
                foreach (var component in components)
                {
                    // Invalid component
                    if (component == null) continue;

                    var type = component.GetType();

                    // Reject DebugInspector
                    if (type == typeof(DebugInspector)) continue;

                    // Pre-existing component
                    if (TryGetHandle(component, out var handle)) continue;

                    // Discovering a new component
                    handle = new InspectedHandle(owner, type);
                    handles.Add(handle);
                }
            }

            private bool TryGetHandle(Component component, out InspectedHandle inspectedHandle)
            {
                inspectedHandle = null;
                foreach (var handle in handles)
                {
                    if (handle.InstanceHandle.Instance == component)
                    {
                        inspectedHandle = handle;
                        break;
                    }
                }

                return inspectedHandle != null;
            }
        }

        [Serializable]
        internal class InspectedItemBase
        {
            [SerializeField] public bool enabled;
            [SerializeField] protected string typeName;

            public bool Valid { get; protected set; }

            public bool Visible => Valid && enabled;
        }

        [Serializable]
        internal class InspectedHandle : InspectedItemBase
        {
            [SerializeField] public List<InspectedMember> inspectedMembers = new();

            public InstanceHandle InstanceHandle { get; private set; }

            public Type Type { get; private set; }

            public InspectedHandle(DebugInspector owner, Type type)
            {
                enabled = false;
                typeName = type.AssemblyQualifiedName;

                Initialize(owner);
            }

            public void Initialize(DebugInspector owner)
            {
                Valid = false;

                Type = System.Type.GetType(typeName);
                if (Type == null)
                {
                    return;
                }

                var component = owner.GetComponent(Type);
                if (component == null)
                {
                    return;
                }

                InstanceHandle = new InstanceHandle(Type, component);

                // Initialize Members
                foreach (var member in inspectedMembers)
                {
                    member.Initialize();
                }

                // Search for new members
                var iterationType = Type;
                while (iterationType != null && iterationType != typeof(Component) && iterationType != typeof(MonoBehaviour))
                {
                    var members = iterationType.GetMembers(InspectedMember.Flags);
                    foreach (var member in members)
                    {
                        if (TryGetMember(member, out var inspectedMember)) continue;

                        if (!member.IsCompatibleWithDebugInspector()) continue;

                        inspectedMember = new InspectedMember(member);
                        inspectedMembers.Add(inspectedMember);
                    }

                    iterationType = iterationType.BaseType;
                }

                Valid = true;
            }

            private bool TryGetMember(MemberInfo memberInfo, out InspectedMember inspectedMember)
            {
                inspectedMember = null;
                foreach (var member in inspectedMembers)
                {
                    if (member.MemberInfo == memberInfo)
                    {
                        inspectedMember = member;
                        break;
                    }
                }

                return inspectedMember != null;
            }
        }

        [Serializable]
        internal class InspectedMember : InspectedItemBase
        {
            internal const BindingFlags Flags = BindingFlags.Instance |
                                                BindingFlags.Static |
                                                BindingFlags.Public |
                                                BindingFlags.NonPublic |
                                                BindingFlags.DeclaredOnly;

            [SerializeField] public DebugMember attribute;
            [SerializeField] private string memberName;

            public MemberInfo MemberInfo { get; private set; }

            public InspectedMember(MemberInfo member)
            {
                enabled = false;
                typeName = member.DeclaringType?.AssemblyQualifiedName;
                memberName = member.Name;
                attribute = new DebugMember();

                Initialize();
            }

            public void Initialize()
            {
                Valid = false;

                var declaringType = System.Type.GetType(typeName);
                if (declaringType == null)
                {
                    return;
                }

                var member = declaringType.GetMember(memberName, Flags);
                if (member.Length == 0)
                {
                    return;
                }

                MemberInfo = member[0];

                Valid = true;
            }
        }

        [SerializeField] private InspectionRegistry registry = new InspectionRegistry();

        internal InspectionRegistry Registry => registry;

        private void OnValidate()
        {
            Initialize();
        }

        private void Initialize()
        {
            registry.Initialize(this);
        }

        private void OnEnable()
        {
            Initialize();

            if (Application.isPlaying)
            {
                DebugInspectorManager.Instance.RegisterInspector(this);
            }
        }

        private void OnDisable()
        {
            DebugInspectorManager.Instance.UnregisterInspector(this);
        }
    }
}

#endif // OVR_INTERNAL_CODE // IMMERSIVE DEBUGGER IS INTERNAL
