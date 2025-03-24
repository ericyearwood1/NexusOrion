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
using System.Text;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using static OVRTelemetry;

internal struct OVRTelemetryMarker : IDisposable
{
    internal struct OVRTelemetryMarkerState
    {
        public bool Sent { get; set; }
        public OVRPlugin.Qpl.ResultType Result { get; set; }

        public OVRTelemetryMarkerState(bool sent, OVRPlugin.Qpl.ResultType result)
        {
            Result = result;
            Sent = sent;
        }
    }


#if OVRPLUGIN_TESTING
    public ulong Handle { get; }
    private readonly Registry _registry;
#endif // OVRPLUGIN_TESTING

#if OVRPLUGIN_TESTING
    private OVRTelemetryMarkerState State
    {
        get => _registry.GetState(this);
        set => _registry.SetState(this, value);
    }
#else
    private OVRTelemetryMarkerState State { get; set; }
#endif // OVRPLUGIN_TESTING

    public bool Sent => State.Sent;
    public OVRPlugin.Qpl.ResultType Result => State.Result;

    public int MarkerId { get; }
    public int InstanceKey { get; }

    private readonly TelemetryClient _client;

    public OVRTelemetryMarker(
        int markerId,
        int instanceKey = OVRPlugin.Qpl.DefaultInstanceKey,
        long timestampMs = OVRPlugin.Qpl.AutoSetTimestampMs)
        : this(
            OVRTelemetry.Client,
#if OVRPLUGIN_TESTING
            OVRTelemetry.CurrentRegistry,
#endif // OVRPLUGIN_TESTING
            markerId,
            instanceKey,
            timestampMs)
    {
    }

    internal OVRTelemetryMarker(
        TelemetryClient client,
#if OVRPLUGIN_TESTING
        Registry registry,
#endif // OVRPLUGIN_TESTING
        int markerId,
        int instanceKey = OVRPlugin.Qpl.DefaultInstanceKey,
        long timestampMs = OVRPlugin.Qpl.AutoSetTimestampMs)
    {
        MarkerId = markerId;
        InstanceKey = instanceKey;
        _client = client;
#if OVRPLUGIN_TESTING
        Handle = (ulong)markerId << 32 | (uint)InstanceKey;
        _registry = registry;
#endif // OVRPLUGIN_TESTING
        State = new OVRTelemetryMarkerState(false, OVRPlugin.Qpl.ResultType.Success);

        _client.MarkerStart(markerId, instanceKey, timestampMs);
    }

    public OVRTelemetryMarker SetResult(OVRPlugin.Qpl.ResultType result)
    {
        State = new OVRTelemetryMarkerState(Sent, result);
        return this;
    }

    public OVRTelemetryMarker AddAnnotation(string annotationKey, string annotationValue)
    {
#if OVRPLUGIN_TESTING
        _registry.OnAnnotation(this, annotationKey, annotationValue);
#endif // OVRPLUGIN_TESTING
        _client.MarkerAnnotation(MarkerId, annotationKey, annotationValue, InstanceKey);
        return this;
    }

    public OVRTelemetryMarker AddAnnotation(string annotationKey, bool annotationValue)
    {
#if OVRPLUGIN_TESTING
        _registry.OnAnnotation(this, annotationKey, OVRPlugin.Qpl.Variant.From(annotationValue));
#endif // OVRPLUGIN_TESTING
        _client.MarkerAnnotation(MarkerId, annotationKey, annotationValue, InstanceKey);
        return this;
    }

    public OVRTelemetryMarker AddAnnotation(string annotationKey, double annotationValue)
    {
#if OVRPLUGIN_TESTING
        _registry.OnAnnotation(this, annotationKey, OVRPlugin.Qpl.Variant.From(annotationValue));
#endif // OVRPLUGIN_TESTING
        _client.MarkerAnnotation(MarkerId, annotationKey, annotationValue, InstanceKey);
        return this;
    }

    public OVRTelemetryMarker AddAnnotation(string annotationKey, long annotationValue)
    {
#if OVRPLUGIN_TESTING
        _registry.OnAnnotation(this, annotationKey, OVRPlugin.Qpl.Variant.From(annotationValue));
#endif // OVRPLUGIN_TESTING
        _client.MarkerAnnotation(MarkerId, annotationKey, annotationValue, InstanceKey);
        return this;
    }

    public unsafe OVRTelemetryMarker AddAnnotation(string annotationKey, byte** annotationValues, int count)
    {
#if OVRPLUGIN_TESTING
        _registry.OnAnnotation(this, annotationKey, OVRPlugin.Qpl.Variant.From(annotationValues, count));
#endif // OVRPLUGIN_TESTING
        _client.MarkerAnnotation(MarkerId, annotationKey, annotationValues, count, InstanceKey);
        return this;
    }

    public unsafe OVRTelemetryMarker AddAnnotation(string annotationKey, long* annotationValues, int count)
    {
#if OVRPLUGIN_TESTING
        _registry.OnAnnotation(this, annotationKey, OVRPlugin.Qpl.Variant.From(annotationValues, count));
#endif // OVRPLUGIN_TESTING
        _client.MarkerAnnotation(MarkerId, annotationKey, annotationValues, count, InstanceKey);
        return this;
    }

    public unsafe OVRTelemetryMarker AddAnnotation(string annotationKey, double* annotationValues, int count)
    {
#if OVRPLUGIN_TESTING
        _registry.OnAnnotation(this, annotationKey, OVRPlugin.Qpl.Variant.From(annotationValues, count));
#endif // OVRPLUGIN_TESTING
        _client.MarkerAnnotation(MarkerId, annotationKey, annotationValues, count, InstanceKey);
        return this;
    }

    public unsafe OVRTelemetryMarker AddAnnotation(string annotationKey, OVRPlugin.Bool* annotationValues, int count)
    {
#if OVRPLUGIN_TESTING
        _registry.OnAnnotation(this, annotationKey, OVRPlugin.Qpl.Variant.From(annotationValues, count));
#endif // OVRPLUGIN_TESTING
        _client.MarkerAnnotation(MarkerId, annotationKey, annotationValues, count, InstanceKey);
        return this;
    }

    public OVRTelemetryMarker AddAnnotationIfNotNullOrEmpty(string annotationKey, string annotationValue)
    {
        return string.IsNullOrEmpty(annotationValue) ? this : AddAnnotation(annotationKey, annotationValue);
    }

    private static string _applicationIdentifier;
    private static string ApplicationIdentifier => _applicationIdentifier ??= Application.identifier;

    private static string _unityVersion;
    private static string UnityVersion => _unityVersion ??= Application.unityVersion;

    public OVRTelemetryMarker Send()
    {
#if OVR_INTERNAL_CODE
        // We won't event send the payload if not internal
        AddAnnotation(OVRTelemetryConstants.OVRManager.AnnotationTypes.Internal, "true");
#endif // OVR_INTERNAL_CODE

        AddAnnotation(OVRTelemetryConstants.OVRManager.AnnotationTypes.ProjectName, ApplicationIdentifier);
        AddAnnotation(OVRTelemetryConstants.OVRManager.AnnotationTypes.ProjectGuid, OVRRuntimeSettings.Instance.TelemetryProjectGuid);
        AddAnnotation(OVRTelemetryConstants.OVRManager.AnnotationTypes.EngineVersion, UnityVersion);

#if OVRPLUGIN_TESTING
        _registry.OnSend(this);
#endif // OVRPLUGIN_TESTING
        State = new OVRTelemetryMarkerState(true, Result);
        _client.MarkerEnd(MarkerId, Result, InstanceKey);

#if OVRPLUGIN_TESTING
        if (OVRRuntimeSettings.Instance.TelemetryLogged)
        {
            Debug.Log($"[Telemetry] Marker sent : {this}");
        }
#endif
        return this;
    }

    public OVRTelemetryMarker SendIf(bool condition)
    {
        if (condition)
        {
            return Send();
        }

        State = new OVRTelemetryMarkerState(true, Result);
        return this;
    }

    public OVRTelemetryMarker AddPoint(OVRTelemetry.MarkerPoint point)
    {
        _client.MarkerPointCached(MarkerId, point.NameHandle, InstanceKey);
        return this;
    }

    public OVRTelemetryMarker AddPoint(string name)
    {
#if OVRPLUGIN_TESTING
        _registry.OnPointData(this, name);
#endif
        _client.MarkerPoint(MarkerId, name, InstanceKey);
        return this;
    }

    public unsafe OVRTelemetryMarker AddPoint(string name, OVRPlugin.Qpl.Annotation.Builder annotationBuilder)
    {
        using var array = annotationBuilder.ToNativeArray();
        return AddPoint(name, (OVRPlugin.Qpl.Annotation*)array.GetUnsafeReadOnlyPtr(), array.Length);
    }

    public unsafe OVRTelemetryMarker AddPoint(string name, OVRPlugin.Qpl.Annotation* annotations, int annotationCount)
    {
#if OVRPLUGIN_TESTING
        _registry.OnPointData(this, name, annotations, annotationCount);
#endif
        _client.MarkerPoint(MarkerId, name, annotations, annotationCount, InstanceKey);
        return this;
    }

    public void Dispose()
    {
        if (!Sent)
        {
            Send();
        }
    }

#if OVRPLUGIN_TESTING
    public override string ToString()
    {
        var result = new StringBuilder($"{MarkersAttribute.FindMarkerName(MarkerId)} : {State.Result}");
        var registry = _registry as RecordedRegistry;
        foreach (var annotation in registry.GetAnnotations(this))
        {
            result.Append($"\n{annotation.Key} : {annotation.Value}");
        }
        return result.ToString();
    }
#endif
}
