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

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

#if UNITY_INCLUDE_TESTS
using NUnit.Framework;
#endif

internal static partial class OVRTelemetry
{
    public static bool? OverrideIsActive { get; private set; }

    private static readonly Registry ActiveRegistry = new RecordedRegistry();
    private static readonly RecordedRegistry ActualRegistry = new RecordedRegistry();
    private static readonly RecordedRegistry ExpectedRegistry = new RecordedRegistry();
    public static Registry CurrentRegistry = ActiveRegistry;

    public static void Mock(bool value)
    {
        OverrideIsActive = value;
        CurrentRegistry = ActualRegistry;
        ResetExpectations();
    }

    public static void Unmock()
    {
        OverrideIsActive = null;
        CurrentRegistry = ActiveRegistry;
    }

    public static OVRTelemetryMarker Expect(int markerId,
        int instanceKey = OVRPlugin.Qpl.DefaultInstanceKey,
        long timestampMs = OVRPlugin.Qpl.AutoSetTimestampMs)
    {
        return new OVRTelemetryMarker(InactiveClient, ExpectedRegistry, markerId, instanceKey, timestampMs).Send();
    }

    public static void ResetExpectations()
    {
        ActualRegistry.Reset();
        ExpectedRegistry.Reset();
    }

    public static void TestExpectations()
    {
        ExpectedRegistry.TestInclusion(ActualRegistry);
    }

    internal class RecordedRegistry : Registry
    {
        internal class MarkerData
        {
            public readonly Dictionary<string, object> Annotations = new();

            public readonly Dictionary<string, Dictionary<string, object>> Points = new();

            public void Annotate(string key, string value) => Annotations[key] = value;

            public void Annotate(string key, OVRPlugin.Qpl.Variant value) => Annotations[key] = ToObject(value);

            public unsafe void Point(string name, OVRPlugin.Qpl.Annotation* annotations, int annotationCount)
            {
                Dictionary<string, object> dict = null;
                if (annotations != null)
                {
                    dict = new Dictionary<string, object>();
                    for (var i = 0; i < annotationCount; i++)
                    {
                        dict[annotations[i].KeyStr] = ToObject(annotations[i].Value);
                    }
                }

                Points[name] = dict;
            }
        }

        readonly Queue<OVRTelemetryMarker> _queue = new();

        readonly Dictionary<ulong, MarkerData> _markers = new();

        static unsafe U[] ToArray<T, U>(T* values, int count, Func<T, U> transformer) where T : unmanaged
        {
            var array = new U[count];
            for (var i = 0; i < count; i++)
            {
                array[i] = transformer(values[i]);
            }
            return array;
        }

        static unsafe T[] ToArray<T>(T* values, int count) where T : unmanaged
            => ToArray(values, count, x => x);

        static unsafe object ToObject(OVRPlugin.Qpl.Variant variant)
        {
            switch (variant.Type)
            {
                case OVRPlugin.Qpl.VariantType.Bool: return variant.BoolValue == OVRPlugin.Bool.True;
                case OVRPlugin.Qpl.VariantType.Double: return variant.DoubleValue;
                case OVRPlugin.Qpl.VariantType.Int: return variant.LongValue;
                case OVRPlugin.Qpl.VariantType.String: return Marshal.PtrToStringUTF8(new IntPtr(variant.StringValue));
                case OVRPlugin.Qpl.VariantType.BoolArray: return ToArray(variant.BoolValues, variant.Count,
                        x => x == OVRPlugin.Bool.True);
                case OVRPlugin.Qpl.VariantType.DoubleArray: return ToArray(variant.DoubleValues, variant.Count);
                case OVRPlugin.Qpl.VariantType.IntArray: return ToArray(variant.LongValues, variant.Count);
                case OVRPlugin.Qpl.VariantType.StringArray:
                {
                    var array = new string[variant.Count];
                    for (var i = 0; i < variant.Count; i++)
                    {
                        array[i] = Marshal.PtrToStringUTF8(new IntPtr(variant.StringValues[i]));
                    }
                    return array;
                }
                default:
                    throw new NotSupportedException();
            }
        }

        public void Reset()
        {
            _markers.Clear();
            _queue.Clear();
        }

        bool TryGetMarkerData(OVRTelemetryMarker marker, out MarkerData markerData)
            => _markers.TryGetValue(marker.Handle, out markerData);

        MarkerData GetOrCreateMarkerData(OVRTelemetryMarker marker)
        {
            if (!_markers.TryGetValue(marker.Handle, out var markerData))
            {
                markerData = new();
                _markers[marker.Handle] = markerData;
            }

            return markerData;
        }

        internal Dictionary<string, object> GetAnnotations(OVRTelemetryMarker marker)
            => GetOrCreateMarkerData(marker).Annotations;

        public override void OnAnnotation(OVRTelemetryMarker marker, string annotationKey, string annotationValue)
        {
            base.OnAnnotation(marker, annotationKey, annotationValue);
            GetOrCreateMarkerData(marker).Annotate(annotationKey, annotationValue);
        }

        public override void OnAnnotation(OVRTelemetryMarker marker, string annotationKey, OVRPlugin.Qpl.Variant annotationValue)
        {
            base.OnAnnotation(marker, annotationKey, annotationValue);
            GetOrCreateMarkerData(marker).Annotate(annotationKey, annotationValue);
        }

        public override unsafe void OnPointData(OVRTelemetryMarker marker, string name, OVRPlugin.Qpl.Annotation* annotations,
            int annotationCount)
        {
            base.OnPointData(marker, name, annotations, annotationCount);
            GetOrCreateMarkerData(marker).Point(name, annotations, annotationCount);
        }

        public override void OnSend(OVRTelemetryMarker marker)
        {
            base.OnSend(marker);
            _queue.Enqueue(marker);
        }

        public void TestInclusion(RecordedRegistry rhsRecord)
        {
            if (rhsRecord == null)
            {
                return;
            }

            var lhsQueue = _queue;
            var rhsQueue = rhsRecord._queue;
            while (lhsQueue.Count > 0)
            {
                var expectedMarker = lhsQueue.Dequeue();

                var actualMarker = rhsQueue.Dequeue();
                while (rhsQueue.Count > 0 && expectedMarker.MarkerId != actualMarker.MarkerId)
                {
                    actualMarker = rhsQueue.Dequeue();
                }

                TestInclusion(this, expectedMarker, rhsRecord, actualMarker);
            }
        }

        private static void TestInclusion(RecordedRegistry expectedRegistry, OVRTelemetryMarker expected,
            RecordedRegistry actualRegistry, OVRTelemetryMarker actual)
        {
            if (actualRegistry == null || expectedRegistry == null)
            {
                return;
            }

#if UNITY_INCLUDE_TESTS
            Assert.That(actual.Sent);
            Assert.That(actual.MarkerId, Is.EqualTo(expected.MarkerId), "MarkerIds do not match.");
            Assert.That(actual.Result, Is.EqualTo(expected.Result));

            void AssertActualAnnotationsHaveExpectedAnnotations(Dictionary<string, object> actualAnnotations,
                Dictionary<string, object> expectedAnnotations)
            {
                if (expectedAnnotations == null) return;

                foreach (var (key, expectedValue) in expectedAnnotations)
                {
                    Assert.That(actualAnnotations.TryGetValue(key, out var actualValue), $"Missing expected annotation '{key}'");
                    Assert.That(actualValue, Is.EqualTo(expectedValue), $"Annotation key={key}");
                }
            }

            if (!expectedRegistry.TryGetMarkerData(expected, out var expectedMarkerData)) return;

            Assert.That(actualRegistry.TryGetMarkerData(actual, out var actualMarkerData));

            // Annotations
            AssertActualAnnotationsHaveExpectedAnnotations(actualMarkerData.Annotations, expectedMarkerData.Annotations);

            // Annotations on point data
            foreach (var (name, expectedAnnotations) in expectedMarkerData.Points)
            {
                Assert.That(actualMarkerData.Points.TryGetValue(name, out var actualAnnotations),
                    $"Missing expected marker point '{name}'");
                AssertActualAnnotationsHaveExpectedAnnotations(actualAnnotations, expectedAnnotations);
            }
#endif
        }
    }
}

#endif // OVRPLUGIN_TESTING
