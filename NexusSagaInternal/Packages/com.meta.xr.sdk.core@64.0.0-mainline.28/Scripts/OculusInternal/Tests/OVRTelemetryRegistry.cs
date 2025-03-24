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


#if OVRPLUGIN_TESTING

using System.Collections.Generic;

internal static partial class OVRTelemetry
{
    internal class Registry
    {
        private readonly Dictionary<ulong, OVRTelemetryMarker.OVRTelemetryMarkerState> _statesRegistry =
            new Dictionary<ulong, OVRTelemetryMarker.OVRTelemetryMarkerState>();

        public OVRTelemetryMarker.OVRTelemetryMarkerState GetState(OVRTelemetryMarker marker)
        {
            if (!_statesRegistry.TryGetValue(marker.Handle, out var state))
            {
                state = new OVRTelemetryMarker.OVRTelemetryMarkerState(false, OVRPlugin.Qpl.ResultType.Success);
                _statesRegistry[marker.Handle] = state;
            }

            return state;
        }

        public void SetState(OVRTelemetryMarker marker, OVRTelemetryMarker.OVRTelemetryMarkerState state)
        {
            _statesRegistry[marker.Handle] = state;
        }

        public virtual void OnAnnotation(OVRTelemetryMarker marker, string annotationKey, string annotationValue) { }
        public virtual void OnAnnotation(OVRTelemetryMarker marker, string annotationKey, OVRPlugin.Qpl.Variant annotationValue) { }

        public unsafe void OnPointData(OVRTelemetryMarker marker, string name)
            => OnPointData(marker, name, null, 0);

        public virtual unsafe void OnPointData(OVRTelemetryMarker marker, string name,
            OVRPlugin.Qpl.Annotation* annotations, int annotationCount) { }

        public virtual void OnSend(OVRTelemetryMarker marker) { }
    }
}

#endif
