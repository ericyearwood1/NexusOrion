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

using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using static OVRTelemetry;

internal class OVRTelemetryTests : OVRPluginEditModeTest
{
    protected override bool TelemetryMockedOut => false;

    private FakeOvrPlugin84 _fakeOVRPlugin84;
    private const int markerId = 987654;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        _fakeOVRPlugin84 = new FakeOvrPlugin84();
        OVRPlugin.OVRP_1_84_0.mockObj = _fakeOVRPlugin84;

        OVRTelemetry.Mock(value: true);
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        OVRPlugin.OVRP_1_84_0.mockObj = new OVRPlugin.OVRP_1_84_0_TEST();

        OVRTelemetry.Unmock();

        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryCallStartAndEnd()
    {
        using (new OVRTelemetryMarker(markerId))
        {
        }

        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerStartCalls.Count);
        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerEndCalls.Count);
        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerAnnotationCalls.Count);

        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerStartCalls[markerId]);
        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerEndCalls[markerId]);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryCallSendEventSuccess()
    {
        OVRTelemetry.SendEvent(markerId);

        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerStartCalls.Count);
        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerEndCalls.Count);
        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerAnnotationCalls.Count);

        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerStartCalls[markerId]);
        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerEndCalls[markerId]);
        Assert.AreEqual(OVRPlugin.Qpl.ResultType.Success, _fakeOVRPlugin84.MarkerEndResults[markerId]);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryCallSendEventFailure()
    {
        OVRTelemetry.SendEvent(markerId, OVRPlugin.Qpl.ResultType.Fail);

        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerStartCalls.Count);
        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerEndCalls.Count);
        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerAnnotationCalls.Count);

        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerStartCalls[markerId]);
        Assert.AreEqual(1, _fakeOVRPlugin84.MarkerEndCalls[markerId]);
        Assert.AreEqual(OVRPlugin.Qpl.ResultType.Fail, _fakeOVRPlugin84.MarkerEndResults[markerId]);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryDefaultSuccess()
    {
        using (new OVRTelemetryMarker(markerId))
        {
        }

        Assert.True(_fakeOVRPlugin84.ActionId.HasValue);
        Assert.AreEqual(OVRPlugin.Qpl.ResultType.Success, _fakeOVRPlugin84.ActionId.Value);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetrySetAction()
    {
        using (var marker = new OVRTelemetryMarker(markerId))
        {
            marker.SetResult(OVRPlugin.Qpl.ResultType.Cancel);
        }

        Assert.True(_fakeOVRPlugin84.ActionId.HasValue);
        Assert.AreEqual(OVRPlugin.Qpl.ResultType.Cancel, _fakeOVRPlugin84.ActionId.Value);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryAddPoints()
    {
        string[] points = { "a", "b", "c" };
        var markerPoints = points.Select(point => new OVRTelemetry.MarkerPoint(point)).ToArray();

        using (var marker = new OVRTelemetryMarker(markerId))
        {
            foreach (var point in markerPoints)
            {
                marker.AddPoint(point);
            }
        }

        Assert.AreEqual(points.Length, _fakeOVRPlugin84.MarkerPoints.Count);
        for (var i = 0; i < points.Length; i++)
        {
            Assert.AreEqual(markerPoints[i].NameHandle, _fakeOVRPlugin84.MarkerPoints[i]);
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryAddAnnotations()
    {
        var annotations = new Dictionary<string, string>()
        {
            { "a", "b" },
            { "c", "d" },
            { "e", "f" }
        };

        using (var marker = new OVRTelemetryMarker(markerId))
        {
            foreach (var annotation in annotations)
            {
                marker.AddAnnotation(annotation.Key, annotation.Value);
            }
        }
#if OVR_INTERNAL_CODE
        annotations.Add(OVRTelemetryConstants.OVRManager.AnnotationTypes.Internal, "true");
#endif
        annotations.Add(OVRTelemetryConstants.OVRManager.AnnotationTypes.ProjectName, Application.identifier);
        annotations.Add(OVRTelemetryConstants.OVRManager.AnnotationTypes.ProjectGuid, OVRRuntimeSettings.Instance.TelemetryProjectGuid);
        annotations.Add(OVRTelemetryConstants.OVRManager.AnnotationTypes.EngineVersion, Application.unityVersion);

        Assert.AreEqual(annotations, _fakeOVRPlugin84.MarkerAnnotations);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryMarkerPointCache()
    {
        const string pointName = "test point name";
        int nameHandle;
        new OVRPlugin.OVRP_1_92_0_TEST().ovrp_QplSetConsent(OVRPlugin.Bool.True);

        using (var point = new OVRTelemetry.MarkerPoint(pointName))
        {
            Assert.AreEqual(1, _fakeOVRPlugin84.CreateMarkerHandleCalls.Count);
            Assert.AreEqual(1, _fakeOVRPlugin84.CreateMarkerHandleCalls[pointName]);
            Assert.Greater(point.NameHandle, 0);
            nameHandle = point.NameHandle;

            using (var marker = new OVRTelemetryMarker(markerId))
            {
                marker.AddPoint(point);
            }

            Assert.AreEqual(1, _fakeOVRPlugin84.MarkerPointCachedCalls.Count);
            Assert.AreEqual(1, _fakeOVRPlugin84.MarkerPointCachedCalls[markerId]);
        }

        Assert.AreEqual(1, _fakeOVRPlugin84.DestroyMarkerHandleCalls.Count);
        Assert.AreEqual(1, _fakeOVRPlugin84.DestroyMarkerHandleCalls[nameHandle]);
        new OVRPlugin.OVRP_1_92_0_TEST().ovrp_QplSetConsent(OVRPlugin.Bool.False);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryNullClient()
    {
        const string pointName = "test point name";
        OVRTelemetry.Mock(value: false);

        using (var marker = new OVRTelemetryMarker(markerId))
        {
            using var point = new OVRTelemetry.MarkerPoint(pointName);
            marker.AddPoint(point);
        }

        Assert.AreEqual(0, _fakeOVRPlugin84.MarkerStartCalls.Count);
        Assert.AreEqual(0, _fakeOVRPlugin84.MarkerEndCalls.Count);
        Assert.AreEqual(0, _fakeOVRPlugin84.MarkerPointCachedCalls.Count);
        Assert.AreEqual(0, _fakeOVRPlugin84.CreateMarkerHandleCalls.Count);
        Assert.AreEqual(0, _fakeOVRPlugin84.DestroyMarkerHandleCalls.Count);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryMarkersAreValid()
    {
        var hashSet = new HashSet<int>();
        var types = MarkersAttribute.FindTypesWithMarkersAttribute();
        foreach (var type in types)
        {
            foreach (var field in type.GetFields())
            {
                if (field.FieldType == typeof(int))
                {
                    var id = (int)field.GetValue(null);
                    // No Zero
                    Assert.AreNotEqual(id, 0);

                    // No Duplicate
                    Assert.IsFalse(hashSet.Contains(id));
                    hashSet.Add(id);
                }
            }
        }

        var markerNames = MarkersAttribute.MarkerNames;
        Assert.IsNotNull(markerNames);
        Assert.AreEqual(markerNames.Keys.AsEnumerable(), hashSet.AsEnumerable());
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTelemetryMarkersFindName()
    {
        var types = MarkersAttribute.FindTypesWithMarkersAttribute();
        foreach (var type in types)
        {
            foreach (var field in type.GetFields())
            {
                if (field.FieldType == typeof(int))
                {
                    var id = (int)field.GetValue(null);
                    Assert.AreEqual(field.Name, MarkersAttribute.FindMarkerName(id));
                }
            }
        }
        yield return null;
    }

    private class FakeOvrPlugin84 : OVRPlugin.OVRP_1_84_0_TEST
    {
        public readonly Dictionary<int, int> MarkerStartCalls = new Dictionary<int, int>();
        public readonly Dictionary<int, int> MarkerEndCalls = new Dictionary<int, int>();

        public readonly Dictionary<int, OVRPlugin.Qpl.ResultType> MarkerEndResults =
            new Dictionary<int, OVRPlugin.Qpl.ResultType>();

        public readonly Dictionary<int, int> MarkerAnnotationCalls = new Dictionary<int, int>();
        public readonly Dictionary<int, int> MarkerPointCachedCalls = new Dictionary<int, int>();
        public readonly Dictionary<string, int> CreateMarkerHandleCalls = new Dictionary<string, int>();
        public readonly Dictionary<int, int> DestroyMarkerHandleCalls = new Dictionary<int, int>();
        public readonly List<int> MarkerPoints = new List<int>();
        public readonly Dictionary<string, string> MarkerAnnotations = new Dictionary<string, string>();

        public OVRPlugin.Qpl.ResultType? ActionId;

        public override OVRPlugin.Result ovrp_QplMarkerAnnotation(int markerId, string annotationKey,
            string annotationValue, int instanceKey)
        {
            MarkerAnnotations[annotationKey] = annotationValue;
            RegisterCall(MarkerAnnotationCalls, markerId);
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_QplMarkerEnd(int markerId, OVRPlugin.Qpl.ResultType resultTypeId,
            int instanceKey, long timestampMs)
        {
            ActionId = resultTypeId;
            MarkerEndResults[markerId] = resultTypeId;
            RegisterCall(MarkerEndCalls, markerId);
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_QplMarkerPointCached(int markerId, int nameHandle, int instanceKey,
            long timestampMs)
        {
            RegisterCall(MarkerPointCachedCalls, markerId);
            MarkerPoints.Add(nameHandle);
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_QplMarkerStart(int markerId, int instanceKey, long timestampMs)
        {
            RegisterCall(MarkerStartCalls, markerId);
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_QplCreateMarkerHandle(string name, out int nameHandle)
        {
            RegisterCall(CreateMarkerHandleCalls, name);
            return base.ovrp_QplCreateMarkerHandle(name, out nameHandle);
        }

        public override OVRPlugin.Result ovrp_QplDestroyMarkerHandle(int nameHandle)
        {
            var result = base.ovrp_QplDestroyMarkerHandle(nameHandle);
            RegisterCall(DestroyMarkerHandleCalls, nameHandle);
            return result;
        }

        private static void RegisterCall<T>(IDictionary<T, int> dictionary, T key)
        {
            dictionary.TryGetValue(key, out var currentCalls);
            dictionary[key] = ++currentCalls;
        }
    }
}

#endif
