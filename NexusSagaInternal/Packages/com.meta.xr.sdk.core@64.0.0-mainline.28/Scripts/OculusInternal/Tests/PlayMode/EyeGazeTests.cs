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
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Bool = OVRPlugin.Bool;
using Result = OVRPlugin.Result;
using Quatf = OVRPlugin.Quatf;
using Vector3f = OVRPlugin.Vector3f;

public class EyeGazeTests : OVRPluginPlayModeTest
{
    private FakeOVRPlugin78 fakeOVRPlugin78;
    private FakeOVRPlugin9 fakeOVRPlugin9;

    private OVREyeGaze leftOvrEyeGaze;
    private OVREyeGaze rightOvrEyeGaze;
    private Transform leftEyeTransform;
    private Transform rightEyeTransform;

    private Vector3f leftEyePos => leftEyeTransform.position.ToFlippedZVector3f();
    private Vector3f rightEyePos => rightEyeTransform.position.ToFlippedZVector3f();

    private Quatf leftEyeRot => leftEyeTransform.rotation.ToFlippedZQuatf();
    private Quatf rightEyeRot => rightEyeTransform.rotation.ToFlippedZQuatf();

    private const float no_confidence = 0.0f;
    private const float default_confidence = 0.5f;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        fakeOVRPlugin78 = new FakeOVRPlugin78
        {
            // Enable mock by default
            StartEyeTrackingMock = true,
            StopEyeTrackingMock = true,
            GetEyeTrackingEnabledMock = true,
            GetEyeGazesStateMock = true
        };

        OVRPlugin.OVRP_1_78_0.mockObj = fakeOVRPlugin78;

        fakeOVRPlugin9 = new FakeOVRPlugin9();
        OVRPlugin.OVRP_1_9_0.mockObj = fakeOVRPlugin9;

        // Create Eye Objects
        leftEyeTransform = new GameObject("Left Eye").transform;
        rightEyeTransform = new GameObject("Right Eye").transform;

        // Left Eye Setup
        leftOvrEyeGaze = leftEyeTransform.gameObject.AddComponent<OVREyeGaze>();
        leftOvrEyeGaze.Eye = OVREyeGaze.EyeId.Left;

        // Right Eye Setup
        rightOvrEyeGaze = rightEyeTransform.gameObject.AddComponent<OVREyeGaze>();
        rightOvrEyeGaze.Eye = OVREyeGaze.EyeId.Right;

        // Common Setup
        leftOvrEyeGaze.ConfidenceThreshold = default_confidence;
        rightOvrEyeGaze.ConfidenceThreshold = default_confidence;
        leftOvrEyeGaze.TrackingMode = OVREyeGaze.EyeTrackingMode.TrackingSpace;
        rightOvrEyeGaze.TrackingMode = OVREyeGaze.EyeTrackingMode.TrackingSpace;
        leftOvrEyeGaze.ApplyPosition = true;
        leftOvrEyeGaze.ApplyRotation = true;
        rightOvrEyeGaze.ApplyPosition = true;
        rightOvrEyeGaze.ApplyRotation = true;
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        // Reset
        Object.Destroy(leftEyeTransform.gameObject);
        Object.Destroy(rightEyeTransform.gameObject);

        // Point mockObjs back at original production version of code
        OVRPlugin.OVRP_1_78_0.mockObj = new OVRPlugin.OVRP_1_78_0_TEST();
        OVRPlugin.OVRP_1_9_0.mockObj = new OVRPlugin.OVRP_1_9_0_TEST();

        // reset to null
        OVRPermissionsRequester.fakeIsPermissionGranted = null;

        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator EyeTransformSyncTest_Validity([Values(Bool.True, Bool.False)] Bool isValid)
    {
        fakeOVRPlugin78.IsValid = isValid;

        // Wait to ensure mockObj changes are properly applied.
        yield return null;

        // Remember to convert back to the right coordinate system

        if (isValid == Bool.True)
        {
            Assert.IsTrue(rightEyePos.IsApproximatelyEqual(FakeOVRPlugin78.TestPosRight));
            Assert.IsTrue(leftEyePos.IsApproximatelyEqual(FakeOVRPlugin78.TestPosLeft));

            Assert.IsTrue(rightEyeRot.IsApproximatelyEqual(FakeOVRPlugin78.TestRot));
            Assert.IsTrue(leftEyeRot.IsApproximatelyEqual(FakeOVRPlugin78.TestRot));
        }
        else
        {
            Assert.IsFalse(rightEyePos.IsApproximatelyEqual(FakeOVRPlugin78.TestPosRight));
            Assert.IsFalse(leftEyePos.IsApproximatelyEqual(FakeOVRPlugin78.TestPosLeft));

            Assert.IsFalse(rightEyeRot.IsApproximatelyEqual(FakeOVRPlugin78.TestRot));
            Assert.IsFalse(leftEyeRot.IsApproximatelyEqual(FakeOVRPlugin78.TestRot));
        }
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator EyeTrackingEnabled([Values(Bool.True, Bool.False)] Bool eyeTrackingEnabled,
        [Values(Result.Success, Result.Success_Pending, Result.Failure)]
        Result getEyeTrackingEnabled)
    {
        // As this is a property, different settings in the OVREyeGaze shouldn't change the result.
        // This means that leftOvrEyeGaze.EyeTrackingEnabled should always be equal to rightOvrEyeGaze.EyeTrackingEnabled.

        fakeOVRPlugin78.EyeTrackingEnabled = eyeTrackingEnabled;
        fakeOVRPlugin78.GetEyeTrackingEnabledResult = getEyeTrackingEnabled;

        yield return null;

        Assert.That(leftOvrEyeGaze.EyeTrackingEnabled, Is.EqualTo(rightOvrEyeGaze.EyeTrackingEnabled));

        // True & Success only
        if (eyeTrackingEnabled == Bool.True && getEyeTrackingEnabled == Result.Success)
        {
            Assert.IsTrue(leftOvrEyeGaze.EyeTrackingEnabled);
        }
        else
        {
            Assert.IsFalse(leftOvrEyeGaze.EyeTrackingEnabled);
        }

        // Reset to "good" values
        fakeOVRPlugin78.EyeTrackingEnabled = Bool.True;
        fakeOVRPlugin78.GetEyeTrackingEnabledResult = Result.Success;

        fakeOVRPlugin78.GetEyeTrackingEnabledMock = false;
        fakeOVRPlugin78.StartEyeTrackingMock = false;
        fakeOVRPlugin78.StopEyeTrackingMock = false;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator OVRP_StartStopEyeTracking()
    {
        // Reset to "good" values
        fakeOVRPlugin78.EyeTrackingEnabled = Bool.True;
        fakeOVRPlugin78.GetEyeTrackingEnabledResult = Result.Success;

        // Disable the Mock OVRP
        fakeOVRPlugin78.GetEyeTrackingEnabledMock = false;
        fakeOVRPlugin78.StartEyeTrackingMock = false;
        fakeOVRPlugin78.StopEyeTrackingMock = false;

        yield return null;

        // Test start eye tracking
        if (OVRPlugin.StartEyeTracking())
        {
            yield return null;

            // Eye Tracking should be enabled
            Assert.IsTrue(OVRPlugin.eyeTrackingEnabled);
        }

        // Test stop eye tracking
        if (OVRPlugin.StopEyeTracking())
        {
            yield return null;

            // Eye Tracking should be disabled
            Assert.IsFalse(OVRPlugin.eyeTrackingEnabled);
        }
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator OVREyeGazeConfidence([Values(no_confidence, default_confidence)] float confidence)
    {
        fakeOVRPlugin78.IsValid = Bool.True;

        // Set the OVREyeGaze confidence to default_confidence (0.5f)
        leftOvrEyeGaze.ConfidenceThreshold = default_confidence;
        rightOvrEyeGaze.ConfidenceThreshold = default_confidence;

        // Change the eyeGazesStates confidence to confidence (no_confidence (0.0f) OR default_confidence (0.5f))
        fakeOVRPlugin78.EyeGazeConfidence = confidence;

        // Wait to ensure mockObj changes are properly applied.
        yield return null;

        // The transforms should NOT sync as eyeGaze.confidence < OVREyeGaze.ConfidenceThreshold.
        // Remember to convert back to the right coordinate system
        if (confidence < default_confidence)
        {
            Assert.IsFalse(rightEyePos.IsApproximatelyEqual(FakeOVRPlugin78.TestPosRight));
            Assert.IsFalse(leftEyePos.IsApproximatelyEqual(FakeOVRPlugin78.TestPosLeft));

            Assert.IsFalse(rightEyeRot.IsApproximatelyEqual(FakeOVRPlugin78.TestRot));
            Assert.IsFalse(leftEyeRot.IsApproximatelyEqual(FakeOVRPlugin78.TestRot));
        }
        // The transforms SHOULD sync as eyeGaze.confidence >= OVREyeGaze.ConfidenceThreshold.
        // Remember to convert back to the right coordinate system
        else
        {
            Assert.IsTrue(rightEyePos.IsApproximatelyEqual(FakeOVRPlugin78.TestPosRight));
            Assert.IsTrue(leftEyePos.IsApproximatelyEqual(FakeOVRPlugin78.TestPosLeft));

            Assert.IsTrue(rightEyeRot.IsApproximatelyEqual(FakeOVRPlugin78.TestRot));
            Assert.IsTrue(leftEyeRot.IsApproximatelyEqual(FakeOVRPlugin78.TestRot));
        }
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator EyeTracking_PermissionsLogicTest([Values(true, false)] bool hasPermission)
    {
        // override permission logic for test
        OVRPermissionsRequester.fakeIsPermissionGranted = (x) =>
        {
            if (x == OVRPermissionsRequester.Permission.EyeTracking) return hasPermission;

            return true;
        };

        leftOvrEyeGaze.gameObject.SetActive(false);
        rightOvrEyeGaze.gameObject.SetActive(false);

        yield return null;

        leftOvrEyeGaze.gameObject.SetActive(true);
        rightOvrEyeGaze.gameObject.SetActive(true);

        yield return null;

        Assert.AreEqual(hasPermission, leftOvrEyeGaze.enabled);
        Assert.AreEqual(hasPermission, rightOvrEyeGaze.enabled);
    }

    private class FakeOVRPlugin78 : OVRPlugin.OVRP_1_78_0_TEST
    {
        public static readonly Vector3f TestPosLeft = new Vector3f() { x = -0.03f, y = 0.5f, z = 0.35f };
        public static readonly Vector3f TestPosRight = new Vector3f() { x = 0.03f, y = 0.5f, z = 0.35f };

        public static readonly Quatf TestRot = new Quatf()
            { x = 0.482962936f, y = -0.224143922f, z = 0.129409552f, w = 0.836516321f };

        public Bool IsValid;
        public Bool EyeTrackingEnabled;

        public Result GetEyeTrackingEnabledResult;
        public Result StartEyeTrackingResult = Result.Success;
        public Result StopEyeTrackingResult = Result.Success;

        public float EyeGazeConfidence = 0.5f;

        public bool StartEyeTrackingMock = true;
        public bool StopEyeTrackingMock = true;
        public bool GetEyeTrackingEnabledMock = true;
        public bool GetEyeGazesStateMock = true;

        public override Result ovrp_StartEyeTracking()
        {
            return StartEyeTrackingMock ? StartEyeTrackingResult : base.ovrp_StartEyeTracking();
        }

        public override Result ovrp_StopEyeTracking()
        {
            return StopEyeTrackingMock ? StopEyeTrackingResult : base.ovrp_StopEyeTracking();
        }

        public override Result ovrp_GetEyeTrackingEnabled(out Bool eyeTrackingEnabled)
        {
            if (!GetEyeTrackingEnabledMock)
            {
                return base.ovrp_GetEyeTrackingEnabled(out eyeTrackingEnabled);
            }

            eyeTrackingEnabled = this.EyeTrackingEnabled;

            return this.GetEyeTrackingEnabledResult;
        }

        public override Result ovrp_GetEyeGazesState(OVRPlugin.Step stepId, int frameIndex,
            out OVRPlugin.EyeGazesStateInternal eyeGazesState)
        {
            if (!GetEyeGazesStateMock)
            {
                return ovrp_GetEyeGazesState(stepId, frameIndex, out eyeGazesState);
            }

            eyeGazesState = new OVRPlugin.EyeGazesStateInternal();

            eyeGazesState.EyeGazes_0 = new OVRPlugin.EyeGazeState()
            {
                Confidence = EyeGazeConfidence,
                Pose = new OVRPlugin.Posef()
                {
                    Orientation = TestRot,
                    Position = TestPosLeft
                },
                _isValid = this.IsValid
            };

            eyeGazesState.EyeGazes_1 = eyeGazesState.EyeGazes_0;

            eyeGazesState.EyeGazes_1.Pose.Position = TestPosRight;

            return Result.Success;
        }
    }

    private class FakeOVRPlugin9 : OVRPlugin.OVRP_1_9_0_TEST
    {
        public override OVRPlugin.SystemHeadset ovrp_GetSystemHeadsetType()
        {
            return OVRPlugin.SystemHeadset.Meta_Quest_Pro;
        }
    }
}

#endif
