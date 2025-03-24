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
using System.Reflection;

namespace Meta.XR.ImmersiveDebugger.Manager
{
    internal interface IDebugManager
    {
        public void Setup(IDebugUIPanel panel, InstanceCache cache);
        public void ProcessType(Type type);

        public void ProcessTypeFromInspector(Type type, InstanceHandle handle, MemberInfo memberInfo,
            DebugMember memberAttribute);

        // Telemetry
        public string TelemetryAnnotation { get; }
        public int GetCountPerType(Type type);
    }

    internal static class ManagerUtils
    {
        public delegate void RegisterMember<in T>(IMember memberController, T member, UnityEngine.Object instance);
        public static void RebuildInspectorForType<T>(IDebugUIPanel panel, InstanceCache cache, Type type, List<T> members, RegisterMember<T> memberRegistration) where T:MemberInfo
        {
            foreach (var member in members)
            {
                var attribute = member.GetCustomAttribute<DebugMember>();
                if (member.IsStatic())
                {
                    var inspector = panel.RegisterInspector(InstanceHandle.Static(type));
                    var memberController = inspector?.RegisterMember(member);
                    if (memberController != null)
                    {
                        memberController.RegisterAttribute(attribute);
                        memberRegistration.Invoke(memberController, member, null);
                    }
                }
                else
                {
                    var instances = cache.GetCacheDataForClass(type);
                    foreach (var instance in instances)
                    {
                        var inspector = panel.RegisterInspector(instance);
                        var memberController = inspector?.RegisterMember(member);
                        if (memberController != null)
                        {
                            memberController.RegisterAttribute(attribute);
                            memberRegistration.Invoke(memberController, member, instance.Instance);
                        }
                    }
                }
            }
        }
    }
}

#endif // OVR_INTERNAL_CODE
