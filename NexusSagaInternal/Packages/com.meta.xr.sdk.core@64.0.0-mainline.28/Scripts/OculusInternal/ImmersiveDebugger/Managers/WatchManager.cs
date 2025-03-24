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
    internal class WatchManager : IDebugManager
    {
        internal readonly Dictionary<Type, List<MemberInfo>> WatchesDict = new Dictionary<Type, List<MemberInfo>>();
        private IDebugUIPanel _uiPanel;
        private InstanceCache _instanceCache;

        public void Setup(IDebugUIPanel panel, InstanceCache cache)
        {
            _uiPanel = panel;
            _instanceCache = cache;
        }

        public void ProcessType(Type type)
        {
            WatchesDict.Remove(type);

            var membersList = new List<MemberInfo>();
            var members = type.GetMembers(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.GetField | BindingFlags.GetProperty);
            foreach (var member in members)
            {
                var watchAttribute = member.GetCustomAttribute<DebugMember>();
                if (watchAttribute != null)
                {
                    if (member.MemberType == MemberTypes.Property | member.MemberType == MemberTypes.Field)
                    {
                        membersList.Add(member);
                    }
                }
            }
            WatchesDict[type] = membersList;
            ManagerUtils.RebuildInspectorForType(_uiPanel, _instanceCache, type, membersList, (memberController, member, instance) =>
            {
                memberController.RegisterWatch(WatchUtils.Create(member, instance));
            });
        }

        public void ProcessTypeFromInspector(Type type, InstanceHandle handle, MemberInfo memberInfo, DebugMember memberAttribute)
        {
            throw new NotImplementedException();
        }

        public string TelemetryAnnotation => Telemetry.AnnotationType.Watches;
        public int GetCountPerType(Type type)
        {
            WatchesDict.TryGetValue(type, out var actions);
            return actions?.Count ?? 0;
        }
    }
}

#endif // OVR_INTERNAL_CODE
