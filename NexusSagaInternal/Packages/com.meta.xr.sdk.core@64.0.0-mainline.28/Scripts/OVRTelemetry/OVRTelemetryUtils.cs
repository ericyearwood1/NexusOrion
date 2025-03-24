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

using System;
using System.Collections.Generic;
using UnityEngine;

internal static partial class OVRTelemetry
{
    [AttributeUsage(AttributeTargets.Class)]
    internal class MarkersAttribute : Attribute
    {
#if OVRPLUGIN_TESTING
        private static Dictionary<int, string> _markerNames;
        internal static Dictionary<int, string> MarkerNames => _markerNames ??= ComputeMarkerNames();

        internal static List<Type> FindTypesWithMarkersAttribute()
        {
            var typesWithMarkers = new List<Type>();
            var assemblies = AppDomain.CurrentDomain.GetAssemblies();

            foreach (var assembly in assemblies)
            {
                var types = assembly.GetTypes();
                foreach (var type in types)
                {
                    if (type.GetCustomAttributes(typeof(MarkersAttribute), true).Length > 0)
                    {
                        typesWithMarkers.Add(type);
                    }
                }
            }

            return typesWithMarkers;
        }

        internal static Dictionary<int, string> ComputeMarkerNames()
        {
            var markerNames = new Dictionary<int, string>();

            var types = FindTypesWithMarkersAttribute();
            foreach (var type in types)
            {
                foreach (var field in type.GetFields())
                {
                    if (field.FieldType == typeof(int))
                    {
                        var markerId = (int)field.GetValue(null);
                        if (markerId == 0)
                        {
                            Debug.LogWarning($"MarkerId {field.Name} cannot be 0");
                        }
                        else if (!markerNames.TryAdd(markerId, field.Name))
                        {
                            Debug.LogWarning($"Duplicate MarkerId {markerId} : {field.Name}, {markerNames[markerId]}");
                        }
                    }
                }
            }

            return markerNames;
        }

        public static string FindMarkerName(int markerId)
        {
            if (MarkerNames.TryGetValue(markerId, out var result))
            {
                return result;
            }

            return "Unknown";
        }
#endif // OVRPLUGIN_TESTING
    }

    private static string _sdkVersionString;

    public static OVRTelemetryMarker AddSDKVersionAnnotation(this OVRTelemetryMarker marker)
    {
        if (string.IsNullOrEmpty(_sdkVersionString))
        {
            _sdkVersionString = OVRPlugin.version.ToString();
        }

        return marker.AddAnnotation("sdk_version", _sdkVersionString);
    }

    private static string GetPlayModeOrigin() => Application.isPlaying
        ? Application.isEditor ? "Editor Play" : "Build Play"
        : "Editor";

    public static OVRTelemetryMarker AddPlayModeOrigin(this OVRTelemetryMarker marker)
    {
        return marker.AddAnnotation(OVRTelemetryConstants.OVRManager.AnnotationTypes.Origin, GetPlayModeOrigin());
    }
}
