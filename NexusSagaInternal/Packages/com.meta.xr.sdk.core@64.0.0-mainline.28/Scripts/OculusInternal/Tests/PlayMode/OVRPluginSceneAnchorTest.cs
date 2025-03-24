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
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine.TestTools;
using UnityEngine;
using UnityEngine.SceneManagement;

internal class OVRPluginSceneAnchorTest : OVRPluginPlayModeTest
{
    private OVRSceneManager _sceneManager;

    private FakeOVRPlugin_65 _fakeOVRPlugin65;
    private FakeOVRPlugin_79 _fakeOVRPlugin79;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        _fakeOVRPlugin65 = new FakeOVRPlugin_65();
        _fakeOVRPlugin79 = new FakeOVRPlugin_79();

        OVRPlugin.OVRP_1_64_0.mockObj = new FakeOVRPlugin_64();
        OVRPlugin.OVRP_1_65_0.mockObj = _fakeOVRPlugin65;
        OVRPlugin.OVRP_1_72_0.mockObj = new FakeOVRPlugin_72();
        OVRPlugin.OVRP_1_79_0.mockObj = _fakeOVRPlugin79;
        OVRPlugin.OVRP_1_84_0.mockObj = new FakeOVRPlugin_84();

        _fakeOVRPlugin79.SetLocationFlagsValidState(true);
        _fakeOVRPlugin79.SetLocate(Vector3.zero, Quaternion.identity);

        yield return LoadTestScene("Oculus/VR/Scenes/OculusInternal/SceneAnchorTests");
        _sceneManager = UnityEngine.Object.FindAnyObjectByType<OVRSceneManager>();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        OVRPlugin.OVRP_1_84_0.mockObj = new OVRPlugin.OVRP_1_84_0_TEST();
        OVRPlugin.OVRP_1_79_0.mockObj = new OVRPlugin.OVRP_1_79_0_TEST();
        OVRPlugin.OVRP_1_72_0.mockObj = new OVRPlugin.OVRP_1_72_0_TEST();
        OVRPlugin.OVRP_1_64_0.mockObj = new OVRPlugin.OVRP_1_64_0_TEST();
        OVRPlugin.OVRP_1_65_0.mockObj = new OVRPlugin.OVRP_1_65_0_TEST();

        OVRTask<bool>.Clear();
        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestRemoveAnchorOnDestroy()
    {
        var newAnchor = CreateSceneAnchor();
        var handle = newAnchor.Space.Handle;
        GameObject.Destroy(newAnchor.gameObject);

        yield return null;

        // expect OVRSpace to be destroyed
        Assert.AreEqual(1, _fakeOVRPlugin65.GetNumberOfDestroyCallsToSpace(handle));
        Assert.AreEqual(1, _fakeOVRPlugin65.GetNumberOfDestroyCalls());

        yield return null;
    }

    private OVRSceneAnchor CreateSceneAnchor()
    {
        ulong handle = (ulong)UnityEngine.Random.Range(1000, 9999999);
        OVRAnchor anchor = new OVRAnchor(new OVRSpace(handle), Guid.NewGuid());

        return _sceneManager.InstantiateSceneAnchor(anchor, _sceneManager.PlanePrefab);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestRemoveSharedAnchorOnDestroy()
    {
        var firstAnchor = CreateSceneAnchor();
        var handle = firstAnchor.Space.Handle;

        var secondAnchor = GameObject.Instantiate(_sceneManager.PlanePrefab);
        secondAnchor.InitializeFrom(firstAnchor.gameObject.GetComponent<OVRSceneAnchor>());

        GameObject.Destroy(firstAnchor.gameObject);
        yield return null;

        // dont expect OVRSpace to be destroyed, since it's being used by another scene anchor
        Assert.AreEqual(0, _fakeOVRPlugin65.GetNumberOfDestroyCallsToSpace(handle));
        Assert.AreEqual(0, _fakeOVRPlugin65.GetNumberOfDestroyCalls());

        GameObject.Destroy(secondAnchor.gameObject);
        yield return null;

        // expect OVRSpace to be destroyed
        Assert.AreEqual(1, _fakeOVRPlugin65.GetNumberOfDestroyCallsToSpace(handle));
        Assert.AreEqual(1, _fakeOVRPlugin65.GetNumberOfDestroyCalls());

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestAnchorUpdateFrequency()
    {
        int nAnchors = 10;
        var anchors = SpawnNAnchors(nAnchors);

        const int nFrames = 50;
        for (var i = 0; i < nFrames; i++)
        {
            yield return null;
        }

        var expectedUpdateNumber = _sceneManager.MaxSceneAnchorUpdatesPerFrame * nFrames / nAnchors;
        int totalLocateSpaceCalls = 0;
        foreach (var anchor in anchors)
        {
            var locateSpaceCallsCount = _fakeOVRPlugin79.GetLocateSpace2CallsCount(anchor.Space);
            Assert.LessOrEqual(Math.Abs(expectedUpdateNumber - locateSpaceCallsCount), 1);
            totalLocateSpaceCalls += locateSpaceCallsCount;
        }

        Assert.AreEqual(totalLocateSpaceCalls, _fakeOVRPlugin79.GetTotalLocateSpace2CallsCount());

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestAnchorUpdateOnlyWhenValid([Values] bool isValid)
    {
        int nAnchors = 10;
        var anchors = SpawnNAnchors(nAnchors);

        foreach (var anchor in anchors)
        {
            anchor.transform.hasChanged = false;
        }

        _fakeOVRPlugin79.SetLocationFlagsValidState(isValid);
        _fakeOVRPlugin79.SetLocate(Vector3.up, Quaternion.identity);

        var anchorUpdateDictionary = new Dictionary<OVRSceneAnchor, bool>();

        const int nFrames = 50;
        for (var i = 0; i < nFrames; i++)
        {
            yield return null;

            foreach (var anchor in anchors)
            {
                if (anchor.transform.hasChanged)
                {
                    anchorUpdateDictionary[anchor] = true;
                }
            }
        }

        foreach (var anchor in anchors)
        {
            anchorUpdateDictionary.TryGetValue(anchor, out var hasUpdated);
            Assert.AreEqual(isValid, hasUpdated);
        }

        yield return null;
    }

    private List<OVRSceneAnchor> SpawnNAnchors(int nAnchors)
    {
        List<OVRSceneAnchor> anchors = new List<OVRSceneAnchor>();

        for (int i = 0; i < nAnchors; i++)
        {
            var newAnchor = CreateSceneAnchor();
            anchors.Add(newAnchor);
        }

        return anchors;
    }

    private IEnumerator BaseTestUpdateAllAnchors(Action trigger)
    {
        var anchors = SpawnNAnchors(10);

        foreach (var anchor in anchors)
        {
            Assert.AreEqual(1, _fakeOVRPlugin79.GetLocateSpace2CallsCount(anchor.Space));

            // clear all anchor pose caches before moving the camera tracking space
            anchor.ClearPoseCache();
        }

        trigger.Invoke();
        yield return null;

        foreach (var anchor in anchors)
        {
            Assert.GreaterOrEqual(_fakeOVRPlugin79.GetLocateSpace2CallsCount(anchor.Space), 2);
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateAllAnchorsOnCameraMove()
    {
        yield return BaseTestUpdateAllAnchors(() =>
        {
            // move camera tracking space to force all anchors to be updated
            var cameraRigTrackingSpace = GameObject.FindAnyObjectByType<OVRCameraRig>().trackingSpace;
            cameraRigTrackingSpace.position += Vector3.up * 1000;
        });
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateAllAnchorsOnRecenteredPose()
    {
        yield return BaseTestUpdateAllAnchors(() =>
        {
            // recenter pose to force all anchors to be updated
            OVRManager.display.RecenterPose();
        });
    }


    private class FakeOVRPlugin_65 : OVRPlugin.OVRP_1_65_0_TEST
    {
        private Dictionary<ulong, int> _destroyCallsDictionary = new Dictionary<ulong, int>();

        public override OVRPlugin.Result ovrp_DestroySpace(ref ulong space)
        {
            _destroyCallsDictionary[space] = GetNumberOfDestroyCallsToSpace(space) + 1;
            return OVRPlugin.Result.Success;
        }

        public int GetNumberOfDestroyCallsToSpace(ulong space)
        {
            _destroyCallsDictionary.TryGetValue(space, out var count);
            return count;
        }

        public int GetNumberOfDestroyCalls()
        {
            return _destroyCallsDictionary.Sum(x => x.Value);
        }
    }

    private class FakeOVRPlugin_64 : OVRPlugin.OVRP_1_64_0_TEST
    {
        private Dictionary<ulong, int> _locateSpaceCallsDictionary = new Dictionary<ulong, int>();

        public override OVRPlugin.Result ovrp_LocateSpace(ref OVRPlugin.Posef location, ref ulong space,
            OVRPlugin.TrackingOrigin trackingOrigin)
        {
            _locateSpaceCallsDictionary.TryGetValue(space, out var callCount);
            _locateSpaceCallsDictionary[space] = callCount + 1;
            return OVRPlugin.Result.Success;
        }

        public int GetLocateSpaceCallsCount(ulong space)
        {
            _locateSpaceCallsDictionary.TryGetValue(space, out var callCount);
            return callCount;
        }

        public int GetTotalLocateSpaceCallsCount()
        {
            return _locateSpaceCallsDictionary.Sum(x => x.Value);
        }
    }

    private class FakeOVRPlugin_72 : OVRPlugin.OVRP_1_72_0_TEST
    {
        public override OVRPlugin.Result ovrp_EnumerateSpaceSupportedComponents(ref UInt64 space,
            uint componentTypesCapacityInput, out uint componentTypesCountOutput,
            OVRPlugin.SpaceComponentType[] componentTypes)
        {
            componentTypesCountOutput = 1;
            componentTypes[0] = OVRPlugin.SpaceComponentType.Locatable;

            return OVRPlugin.Result.Success;
        }

        public override unsafe OVRPlugin.Result ovrp_EnumerateSpaceSupportedComponents(ref UInt64 space,
            uint componentTypesCapacityInput, out uint componentTypesCountOutput,
            OVRPlugin.SpaceComponentType* componentTypes)
        {
            componentTypesCountOutput = 1;

            if (componentTypesCapacityInput == 0)
            {
                return OVRPlugin.Result.Success;
            }

            if (componentTypes == null)
            {
                return OVRPlugin.Result.Failure_InvalidParameter;
            }

            componentTypes[0] = OVRPlugin.SpaceComponentType.Locatable;
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceComponentStatus(ref ulong space,
            OVRPlugin.SpaceComponentType componentType, out OVRPlugin.Bool enabled,
            out OVRPlugin.Bool changePending)
        {
            enabled = OVRPlugin.Bool.True;
            changePending = OVRPlugin.Bool.True;

            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceBoundingBox3D(ref ulong space, out OVRPlugin.Boundsf bounds)
        {
            bounds = default;
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceBoundingBox2D(ref ulong space, out OVRPlugin.Rectf rect)
        {
            rect = default;
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceSemanticLabels(ref ulong space,
            ref OVRPlugin.SpaceSemanticLabelInternal labelsInternal)
        {
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_QuerySpaces(ref OVRPlugin.SpaceQueryInfo queryInfo, out ulong requestId)
        {
            requestId = 1;
            return OVRPlugin.Result.Success;
        }
    }

    private class FakeOVRPlugin_79 : OVRPlugin.OVRP_1_79_0_TEST
    {
        private readonly Dictionary<ulong, int> _locateSpace2CallsDictionary = new Dictionary<ulong, int>();

        private const OVRPlugin.SpaceLocationFlags ValidFlags = OVRPlugin.SpaceLocationFlags.OrientationValid |
                                                                OVRPlugin.SpaceLocationFlags.PositionValid |
                                                                OVRPlugin.SpaceLocationFlags.OrientationTracked |
                                                                OVRPlugin.SpaceLocationFlags.PositionTracked;

        private OVRPlugin.SpaceLocationFlags _locationFlags = ValidFlags;
        private OVRPlugin.Vector3f Position;
        private OVRPlugin.Quatf Orientation;

        public void SetLocationFlagsValidState(bool valid)
        {
            _locationFlags = valid ? ValidFlags : default;
        }

        public void SetLocate(Vector3 location, Quaternion rotation)
        {
            Position = location.ToVector3f();
            Orientation = rotation.ToQuatf();
        }

#if OVR_INTERNAL_CODE
        public override OVRPlugin.Result ovrp_GetPlaneTrackingSupported(out OVRPlugin.Bool planeTrackingSupported)
        {
            planeTrackingSupported = OVRPlugin.Bool.False;
            return OVRPlugin.Result.Success;
        }
#endif

        public override OVRPlugin.Result ovrp_LocateSpace2(out OVRPlugin.SpaceLocationf location, in ulong space, OVRPlugin.TrackingOrigin trackingOrigin)
        {
            _locateSpace2CallsDictionary.TryGetValue(space, out var callCount);
            _locateSpace2CallsDictionary[space] = callCount + 1;
            location = new OVRPlugin.SpaceLocationf
            {
                pose = new OVRPlugin.Posef { Orientation = Orientation, Position = Position },
                locationFlags = _locationFlags
            };
            return OVRPlugin.Result.Success;
        }

        public int GetLocateSpace2CallsCount(ulong space)
        {
            _locateSpace2CallsDictionary.TryGetValue(space, out var callCount);
            return callCount;
        }

        public int GetTotalLocateSpace2CallsCount()
        {
            return _locateSpace2CallsDictionary.Sum(x => x.Value);
        }
    }

    private class FakeOVRPlugin_84 : OVRPlugin.OVRP_1_84_0_TEST
    {
        // QPL
        public override OVRPlugin.Result ovrp_QplMarkerStart(int markerId, int instanceKey, long timestampMs) =>
            OVRPlugin.Result.Success;

        public override OVRPlugin.Result ovrp_QplMarkerEnd(int markerId, OVRPlugin.Qpl.ResultType resultTypeId,
            int instanceKey, long timestampMs) => OVRPlugin.Result.Success;

        public override OVRPlugin.Result ovrp_QplMarkerPointCached(int markerId, int nameHandle,
            int instanceKey, long timestampMs) => OVRPlugin.Result.Success;

        public override OVRPlugin.Result ovrp_QplMarkerAnnotation(int markerId,
            string annotationKey,
            string annotationValue, int instanceKey) => OVRPlugin.Result.Success;

        public override OVRPlugin.Result ovrp_QplCreateMarkerHandle(string name, out int nameHandle)
        {
            nameHandle = 1;
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_QplDestroyMarkerHandle(int nameHandle) => OVRPlugin.Result.Success;
    }
}

#endif
