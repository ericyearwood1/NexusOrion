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

using System.Collections.Generic;
using Meta.XR.ImmersiveDebugger.Manager;
using Meta.XR.ImmersiveDebugger.Utils;
using static OVRTelemetry;

namespace Meta.XR.ImmersiveDebugger
{
    internal static class Telemetry
    {
        [Markers]
        public static class MarkerId
        {
            public const int SettingsAccessed = 163059815;
            public const int SettingsChanged = 163065143;
            public const int ComponentTracked = 163059554;
            public const int Run = 163061656;
        }

        public enum State
        {
            OnStart,
            OnFocusLost,
            OnDisable
        }

        public enum Method
        {
            Attributes,
            DebugInspector
        }

        public static class AnnotationType
        {
            public const string Origin = "Origin";
            public const string Type = "Type";
            public const string Value = "Value";
            public const string Method = "Method";
            public const string State = "State";
            public const string Instances = "Instances";
            public const string Gizmos = "Gizmos";
            public const string Watches = "Watches";
            public const string Tweaks = "Tweaks";
            public const string Actions = "Actions";
        }

        internal class TelemetryTracker
        {
            private Telemetry.Method _method;
            private InstanceCache _cache;
            private IEnumerable<IDebugManager> _managers;

            public static void Start(Telemetry.Method method,
                IEnumerable<IDebugManager> managers,
                InstanceCache cache,
                DebugManager debugManager)
            {
                var telemetryTracker = new TelemetryTracker(method, managers, cache);
                debugManager.OnStartAction += telemetryTracker.OnStart;
                debugManager.OnFocusLostAction += telemetryTracker.OnFocusLost;
                debugManager.OnDisableAction += telemetryTracker.OnDisable;
            }

            private TelemetryTracker(
                Telemetry.Method method,
                IEnumerable<IDebugManager> managers,
                InstanceCache cache)
            {
                _method = method;
                _cache = cache;
                _managers = managers;
            }

            private void OnStart()
            {
                SendStart();
                SendComponentTracked(State.OnStart);
            }

            private void OnFocusLost()
            {
                SendComponentTracked(State.OnFocusLost);
            }

            private void OnDisable()
            {
                SendComponentTracked(State.OnDisable);
            }

            private void SendStart()
            {
                OVRTelemetry.Start(Telemetry.MarkerId.Run)
                    .AddAnnotation(Telemetry.AnnotationType.Method, _method.ToString())
                    .AddAnnotation(Telemetry.AnnotationType.State, State.OnStart.ToString())
                    .AddPlayModeOrigin()
                    .Send();
            }

            private void SendComponentTracked(State state)
            {
                foreach (var dataPair in _cache.CacheData)
                {
                    var type = dataPair.Key;
                    var instances = dataPair.Value;

                    var instancesCount = instances.Count;
                    if (instancesCount > 0)
                    {
                        var marker = OVRTelemetry.Start(Telemetry.MarkerId.ComponentTracked)
                            .AddPlayModeOrigin()
                            .AddAnnotation(Telemetry.AnnotationType.State, state.ToString())
                            .AddAnnotation(Telemetry.AnnotationType.Method, _method.ToString())
                            .AddAnnotation(Telemetry.AnnotationType.Type, type.FullName)
                            .AddAnnotation(Telemetry.AnnotationType.Instances, instances.Count.ToString());

                        foreach (var manager in _managers)
                        {
                            marker.AddAnnotation(manager.TelemetryAnnotation, manager.GetCountPerType(type).ToString());
                        }

                        marker.Send();
                    }
                }
            }
        }
    }
}

#endif
