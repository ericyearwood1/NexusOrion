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
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Bool = OVRPlugin.Bool;
using Result = OVRPlugin.Result;
using Quatf = OVRPlugin.Quatf;
using Vector3f = OVRPlugin.Vector3f;
using static OVRSkeleton;
using NUnit.Framework.Interfaces;

#region PreSetup

public class setBodyPermissions : NUnitAttribute, IOuterUnityTestAction
{
    public IEnumerator BeforeTest(ITest test)
    {
        Debug.Log("Setting the body tracking permission to false");
        OVRPermissionsRequester.fakeIsPermissionGranted = (permission) =>
        {
            if (permission == OVRPermissionsRequester.Permission.BodyTracking)
                return false;
            return true;
        };

        yield return null;
    }

    public IEnumerator AfterTest(ITest test)
    {
        OVRPermissionsRequester.fakeIsPermissionGranted = null;

        yield return null;
    }
}

#endregion

#region TestSetup

public class BodyTrackingTests : OVRPluginPlayModeTest
{
    private FakeOVRPlugin78 _fakeOvrPlugin78;
    private FakeOVRPlugin84 _fakeOvrPlugin84;
    private FakeOVRPlugin90 _fakeOvrPlugin90;
    private FakeOVRPlugin92 _fakeOvrPlugin92;
    private FakeOVRPluginState _state;
    private OVRBody _ovrBody;
    private OVRSkeleton _ovrSkeleton;
    protected GameObject bodyGameObject;
    public List<GameObject> goList = new List<GameObject>();

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        _state = new FakeOVRPluginState();
        _fakeOvrPlugin78 = new FakeOVRPlugin78(_state);
        OVRPlugin.OVRP_1_78_0.mockObj = _fakeOvrPlugin78;
        _fakeOvrPlugin84 = new FakeOVRPlugin84(_state);
        OVRPlugin.OVRP_1_84_0.mockObj = _fakeOvrPlugin84;
        _fakeOvrPlugin90 = new FakeOVRPlugin90(_state);
        OVRPlugin.OVRP_1_90_0.mockObj = _fakeOvrPlugin90;
        _fakeOvrPlugin92 = new FakeOVRPlugin92(_state);
        OVRPlugin.OVRP_1_92_0.mockObj = _fakeOvrPlugin92;

        // Requested (tracking) joint set is global
        OVRBody.SetRequestedJointSet(OVRPlugin.BodyJointSet.UpperBody);

        bodyGameObject = new GameObject("Avatar Body");
        _ovrBody = bodyGameObject.AddComponent<OVRBody>();
        _ovrSkeleton = bodyGameObject.AddComponent<OVRSkeleton>();
        goList.Add(bodyGameObject);
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        // Point mock objects back at original production version of code
        OVRPlugin.OVRP_1_78_0.mockObj = new OVRPlugin.OVRP_1_78_0_TEST();
        OVRPlugin.OVRP_1_9_0.mockObj = new OVRPlugin.OVRP_1_9_0_TEST();
        _state.PreviousStartStopCall = StateOperation.None;
        goList.ForEach(Object.Destroy);
        goList.Clear();
        yield return base.UnityTearDown();
    }

#endregion

#region BodyTrackingConfidenceTest

    private const float LowConfidence = 0.0f;
    private const float MidConfidence = 0.5f;
    private const float HighConfidence = 1.0f;

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator BodyTrackingState_Confidence(
        [Values(LowConfidence, MidConfidence, HighConfidence)]
        float testConfidence)
    {
        _state.BodyStateConfidence = 0.0f;
        _state.BodyStateConfidence = 0.0f;
        // finding IOVRSkeletonDataProvider through OVRSkeleton to ensure reference is the same
        var dataProvider = _ovrSkeleton.GetComponent<IOVRSkeletonDataProvider>();
        var initData = dataProvider.GetSkeletonPoseData();
        _state.BodyStateConfidence = testConfidence;
        _state.BodyStateConfidence = testConfidence;
        yield return null;
        var newData = dataProvider.GetSkeletonPoseData();
        if (testConfidence == HighConfidence)
        {
            Assert.AreNotEqual(initData, newData);
        }
        else
        {
            Assert.AreEqual(initData, newData);
        }

        yield return null;
    }

#endregion

#region BodyTrackingEnabledTest

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator StartStop_BodyTracking()
    {
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);
        Assert.AreEqual(OVRPlugin.BodyJointSet.UpperBody, _state.PreviousStartedJointSet);
        _ovrBody.enabled = false;
        Assert.AreEqual(StateOperation.Stop, _state.PreviousStartStopCall);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator StartStop_BodyTracking_multi_ovrBody()
    {
        var bodyGameObject2 = new GameObject("Avatar Body");
        var ovrBody2 = bodyGameObject2.AddComponent<OVRBody>();
        Assert.IsTrue(ovrBody2.enabled);
        Assert.IsTrue(_ovrBody.enabled);
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);
        Assert.AreEqual(OVRPlugin.BodyJointSet.UpperBody, _state.PreviousStartedJointSet);

        ovrBody2.enabled = false;

        // We don't call stop since only one OVRBody was disabled
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);
        Assert.AreEqual(OVRPlugin.BodyJointSet.UpperBody, _state.PreviousStartedJointSet);
        _ovrBody.enabled = false;

        // Now, we call stop since all OVRBody instances are disabled
        Assert.AreEqual(StateOperation.Stop, _state.PreviousStartStopCall);


        yield return null;
    }

    [UnityTest, setBodyPermissions]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator BodyTrackingEnabledTest()
    {
        _state.GetBodyTrackingEnabled = Result.Failure;
        Assert.IsFalse(OVRPlugin.bodyTrackingEnabled);

        _state.GetBodyTrackingEnabled = Result.Success;
        Assert.IsTrue(OVRPlugin.bodyTrackingEnabled);

        yield return null;
    }

#endregion

#region BodyTrackingFlags

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator BodyTracking_Flags_Fail()
    {
        _state.LocFlags = 0;
        _state.LocFlags = 0;
        yield return null;
        Assert.IsFalse(_ovrBody.BodyState?.JointLocations[0].OrientationValid);
        Assert.IsFalse(_ovrBody.BodyState?.JointLocations[0].PositionValid);
        Assert.IsFalse(_ovrBody.BodyState?.JointLocations[0].OrientationTracked);
        Assert.IsFalse(_ovrBody.BodyState?.JointLocations[0].PositionTracked);

        yield return null;
    }

#endregion

#region FullBodyTrackingTests

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator SwitchJointSet()
    {
        Assert.IsTrue(_ovrBody.isActiveAndEnabled);
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);
        Assert.AreEqual(OVRPlugin.BodyJointSet.UpperBody, _state.PreviousStartedJointSet);

        OVRBody.SetRequestedJointSet(OVRPlugin.BodyJointSet.FullBody);

        // When running, body tracking is restarted immediately
        Assert.AreEqual(2, _state.StartBodyTrackingCount);
        Assert.AreEqual(OVRPlugin.BodyJointSet.FullBody, _state.PreviousStartedJointSet);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator StartStop_FullBodyTracking()
    {
        bodyGameObject.SetActive(false);
        OVRBody.SetRequestedJointSet(OVRPlugin.BodyJointSet.FullBody);
        bodyGameObject.SetActive(true);

        Assert.IsTrue(_ovrBody.enabled);
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);
        Assert.AreEqual(OVRPlugin.BodyJointSet.FullBody, _state.PreviousStartedJointSet);

        _ovrBody.enabled = false;
        Assert.AreEqual(StateOperation.Stop, _state.PreviousStartStopCall);

        yield return null;
    }

#if OVR_INTERNAL_CODE
    [UnityTest, setBodyPermissions]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator FullBodyTrackingEnabledTest()
    {
        _state.GetFullBodyTrackingEnabled = Result.Failure;
        Assert.IsFalse(OVRPlugin.fullBodyTrackingEnabled);

        _state.GetFullBodyTrackingEnabled = Result.Success;
        Assert.IsTrue(OVRPlugin.fullBodyTrackingEnabled);

        yield return null;
    }
#endif

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator StartStop_FullAndUpperBodyTracking()
    {
        _state.GetBodyTrackingEnabled = Result.Success;

        var bodyGameObject2 = new GameObject("Avatar Body2");
        var ovrBody2 = bodyGameObject2.AddComponent<OVRBody>();
        goList.Add(bodyGameObject2);
        Assert.IsTrue(ovrBody2.enabled);
        Assert.IsTrue(_ovrBody.enabled);
        Assert.AreEqual(1, _state.StartBodyTrackingCount);
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);

        // By default, OVRBody instances are created with the UpperBody joint set
        Assert.AreEqual(OVRPlugin.BodyJointSet.UpperBody, _state.PreviousStartedJointSet);

        // Switching one OVRBody to FullBody doesn't affect the running tracker
        bodyGameObject.SetActive(false);
        _ovrBody.ProvidedSkeletonType = OVRPlugin.BodyJointSet.FullBody;
        bodyGameObject.SetActive(true);
        Assert.AreEqual(1, _state.StartBodyTrackingCount);

        // Switching the new tracking joint set requires restarting
        OVRBody.SetRequestedJointSet(OVRPlugin.BodyJointSet.FullBody);
        Assert.AreEqual(2, _state.StartBodyTrackingCount);
        Assert.AreEqual(OVRPlugin.BodyJointSet.FullBody, _state.PreviousStartedJointSet);
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);

        // Switching the second OVRBody to FullBody should not change anything
        bodyGameObject2.SetActive(false);
        ovrBody2.ProvidedSkeletonType = OVRPlugin.BodyJointSet.FullBody;
        bodyGameObject2.SetActive(true);

        Assert.AreEqual(2, _state.StartBodyTrackingCount);
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);

        // Switching one of the OVRBody instances back to UpperBody shouldn't do anything
        bodyGameObject.SetActive(false);
        _ovrBody.ProvidedSkeletonType = OVRPlugin.BodyJointSet.UpperBody;
        bodyGameObject.SetActive(true);
        Assert.AreEqual(2, _state.StartBodyTrackingCount);
        Assert.AreEqual(OVRPlugin.BodyJointSet.FullBody, _state.PreviousStartedJointSet);
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);


        // Disabling the only full body instance changes nothing
        bodyGameObject2.SetActive(false);
        Assert.AreEqual(2, _state.StartBodyTrackingCount);
        Assert.AreEqual(OVRPlugin.BodyJointSet.FullBody, _state.PreviousStartedJointSet);
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);

        // Switching the new tracking joint set requires restarting
        OVRBody.SetRequestedJointSet(OVRPlugin.BodyJointSet.UpperBody);
        Assert.AreEqual(3, _state.StartBodyTrackingCount);
        Assert.AreEqual(OVRPlugin.BodyJointSet.UpperBody, _state.PreviousStartedJointSet);
        Assert.AreEqual(StateOperation.Start, _state.PreviousStartStopCall);

        yield return null;
    }
#endregion


#region BodyTrackingFlags


    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator BodyFaceEyeTracking_RequestSupported()
    {
        yield return null;
        OVRPermissionsRequester.Request(new[] { OVRPermissionsRequester.Permission.BodyTracking });
        Assert.That(_state.BodyTrackingCount, Is.EqualTo(1));
        OVRPermissionsRequester.Request(new[] { OVRPermissionsRequester.Permission.FaceTracking });
        Assert.That(_state.FaceTrackingCount, Is.EqualTo(1));
        OVRPermissionsRequester.Request(new[] { OVRPermissionsRequester.Permission.EyeTracking });
        Assert.That(_state.EyeTrackingCount, Is.EqualTo(1));
        yield return null;
    }
#endregion

#region BodyTrackingCalibration
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator BodyCalibration_GetCalibrationStatus()
    {
        _state.CalibrationStatus = OVRPlugin.BodyTrackingCalibrationState.Valid;
        yield return null;
        Assert.AreEqual(_ovrBody.GetBodyTrackingCalibrationStatus(), OVRPlugin.BodyTrackingCalibrationState.Valid);

        _state.CalibrationStatus = OVRPlugin.BodyTrackingCalibrationState.Calibrating;
        yield return null;
        Assert.AreEqual(_ovrBody.GetBodyTrackingCalibrationStatus(), OVRPlugin.BodyTrackingCalibrationState.Calibrating);

        _state.CalibrationStatus = OVRPlugin.BodyTrackingCalibrationState.Invalid;
        yield return null;
        Assert.AreEqual(_ovrBody.GetBodyTrackingCalibrationStatus(), OVRPlugin.BodyTrackingCalibrationState.Invalid);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator BodyCalibration_SetCalibrationInfo()
    {
        OVRBody.SuggestBodyTrackingCalibrationOverride(2.0f);
        Assert.AreEqual(_state.CalibrationInfo?.BodyHeight ?? 0.0, 2.0f, 1e-5);

        yield return null;
    }
#endregion

#region TestMock
    // Used to keep track of which state function is called
    internal enum StateOperation { None, Start, Stop }

    internal class FakeOVRPluginState
    {
        public static Vector3f TestPos = new Vector3f() { x = 0.2f, y = 0.3f, z = 0.4f };

        public static Quatf TestRot = new Quatf()
        { x = 0.00439511752f, y = 0.00520915771f, z = 0.00608565286f, w = 0.999958336f };

        public float BodyStateConfidence = 0.5f;
        public Bool BodyStateIsActive = Bool.True;
        public Bool BodyTrackingEnabledValue = Bool.True;
        public Result GetBodyTrackingEnabled;
        public Result StartBodyTracking = Result.Success;
        public Result StopBodyTracking = Result.Success;
        public OVRPlugin.SpaceLocationFlags LocFlags = (OVRPlugin.SpaceLocationFlags)0x00000001;

        public Bool FullBodyTrackingEnabledValue = Bool.True;
        public Result StartBodyTracking2 = Result.Success;
        public Result GetFullBodyTrackingEnabled = Result.Success;

        public StateOperation PreviousStartStopCall = StateOperation.None;
        public OVRPlugin.BodyJointSet PreviousStartedJointSet = OVRPlugin.BodyJointSet.UpperBody;
        public int BodyTrackingCount = 0;
        public int FaceTrackingCount = 0;
        public int EyeTrackingCount = 0;

        public int StartBodyTrackingCount = 0;

        public OVRPlugin.BodyTrackingCalibrationState CalibrationStatus = OVRPlugin.BodyTrackingCalibrationState.Valid;
        public OVRPlugin.BodyTrackingFidelity2 Fidelity = OVRPlugin.BodyTrackingFidelity2.Low;
        public OVRPlugin.BodyTrackingCalibrationInfo? CalibrationInfo = null;
    }

    internal class FakeOVRPlugin92 : OVRPlugin.OVRP_1_92_0_TEST
    {
        private FakeOVRPluginState _state;
        private FakeOVRPlugin84 _v84Plugin;

        public FakeOVRPlugin92(FakeOVRPluginState state)
        {
            _state = state;
        }

        public override Result ovrp_GetBodyState4(OVRPlugin.Step stepId, int frameIndex, out OVRPlugin.BodyState4Internal bodyState)
        {
            bodyState = new OVRPlugin.BodyState4Internal()
            {
                // if greater than 0.5 change to posef values
                Confidence = _state.BodyStateConfidence,
                IsActive = _state.BodyStateIsActive,
                CalibrationStatus = _state.CalibrationStatus,
                Fidelity = _state.Fidelity,
                JointLocation_0 = new OVRPlugin.BodyJointLocation()
                {
                    Pose = new OVRPlugin.Posef()
                    {
                        Orientation = FakeOVRPluginState.TestRot,
                        Position = FakeOVRPluginState.TestPos,
                    },
                    LocationFlags = _state.LocFlags
                }
            };

            return Result.Success;
        }

        public override Result ovrp_SuggestBodyTrackingCalibrationOverride(OVRPlugin.BodyTrackingCalibrationInfo calibrationInfo)
        {
            _state.CalibrationInfo = calibrationInfo;

            return Result.Success;
        }

        public override Result ovrp_ResetBodyTrackingCalibration()
        {
            _state.CalibrationInfo = null;

            return Result.Success;
        }

        public override Result ovrp_RequestBodyTrackingFidelity(OVRPlugin.BodyTrackingFidelity2 fidelity)
        {
            _state.Fidelity = fidelity;

            return Result.Success;
        }
        public override Result ovrp_StartBodyTracking2(OVRPlugin.BodyJointSet jointSet)
        {
            if (_state.PreviousStartStopCall == StateOperation.Start)
            {
                Assert.Fail("StartBodyTracking called twice without stopping in between");
            }
            _state.PreviousStartStopCall = StateOperation.Start;
            _state.PreviousStartedJointSet = jointSet;

            _state.StartBodyTrackingCount++;

            return _state.StartBodyTracking2;
        }
    }

    internal class FakeOVRPlugin90 : OVRPlugin.OVRP_1_90_0_TEST
    {
        private FakeOVRPluginState _state;
        private FakeOVRPlugin84 _v84Plugin;

        public FakeOVRPlugin90(FakeOVRPluginState state)
        {
            _state = state;
        }
#if OVR_INTERNAL_CODE
        public override Result ovrp_GetBodyState3(OVRPlugin.Step stepId, int frameIndex, out OVRPlugin.BodyState3Internal bodyState)
        {
            OVRPlugin.BodyTrackingCalibrationStatus calibrationStatus = (OVRPlugin.BodyTrackingCalibrationStatus)(-1);
            switch (_state.CalibrationStatus)
            {
                case OVRPlugin.BodyTrackingCalibrationState.Valid:
                    calibrationStatus = OVRPlugin.BodyTrackingCalibrationStatus.Valid;
                    break;
                case OVRPlugin.BodyTrackingCalibrationState.Calibrating:
                    calibrationStatus = OVRPlugin.BodyTrackingCalibrationStatus.Calibrating;
                    break;
                case OVRPlugin.BodyTrackingCalibrationState.Invalid:
                    calibrationStatus = OVRPlugin.BodyTrackingCalibrationStatus.Invalid;
                    break;
            }
            bodyState = new OVRPlugin.BodyState3Internal()
            {
                // if greater than 0.5 change to posef values
                Confidence = _state.BodyStateConfidence,
                IsActive = _state.BodyStateIsActive,
                CalibrationStatus = calibrationStatus,
                JointLocation_0 = new OVRPlugin.BodyJointLocation()
                {
                    Pose = new OVRPlugin.Posef()
                    {
                        Orientation = FakeOVRPluginState.TestRot,
                        Position = FakeOVRPluginState.TestPos,
                    },
                    LocationFlags = _state.LocFlags
                }
            };

            return Result.Success;
        }

        public override Result ovrp_SuggestBodyTrackingCalibration(OVRPlugin.BodyTrackingCalibrationInfo calibrationInfo)
        {
            _state.CalibrationInfo = calibrationInfo;

            return Result.Success;
        }
#endif
    }

    internal class FakeOVRPlugin84 : OVRPlugin.OVRP_1_84_0_TEST
    {
        private FakeOVRPluginState _state;

        public FakeOVRPlugin84(FakeOVRPluginState state)
        {
            this._state = state;
        }

#if OVR_INTERNAL_CODE

        public override Result ovrp_GetBodyState2(OVRPlugin.Step stepId, int frameIndex,
            out OVRPlugin.BodyState2Internal bodyState)
        {
            bodyState = new OVRPlugin.BodyState2Internal()
            {
                // if greater than 0.5 change to posef values
                Confidence = _state.BodyStateConfidence,
                IsActive = _state.BodyStateIsActive,
                JointLocation_0 = new OVRPlugin.BodyJointLocation()
                {
                    Pose = new OVRPlugin.Posef()
                    {
                        Orientation = FakeOVRPluginState.TestRot,
                        Position = FakeOVRPluginState.TestPos,
                    },
                    LocationFlags = _state.LocFlags
                }
            };

            return Result.Success;
        }
        public override Result ovrp_GetFullBodyTrackingEnabled(out Bool fullBodyTrackingEnabled)
        {
            fullBodyTrackingEnabled = _state.FullBodyTrackingEnabledValue;

            return _state.GetFullBodyTrackingEnabled;
        }
#endif
    }

    internal class FakeOVRPlugin78 : OVRPlugin.OVRP_1_78_0_TEST
    {
        private FakeOVRPluginState _state;
        public FakeOVRPlugin78(FakeOVRPluginState state)
        {
            this._state = state;
        }

        public override Result ovrp_StartBodyTracking()
        {
            if (_state.PreviousStartStopCall == StateOperation.Start)
            {
                Assert.Fail("StartBodyTracking called twice without stopping in between");
            }
            _state.PreviousStartStopCall = StateOperation.Start;
            _state.StartBodyTrackingCount++;

            return _state.StartBodyTracking;
        }

        public override Result ovrp_StopBodyTracking()
        {
            _state.PreviousStartStopCall = StateOperation.Stop;

            return _state.StopBodyTracking;
        }

        public override Result ovrp_GetBodyTrackingEnabled(out Bool bodyTrackingEnabled)
        {
            bodyTrackingEnabled = _state.BodyTrackingEnabledValue;

            return _state.GetBodyTrackingEnabled;
        }

        public override Result ovrp_GetBodyState(OVRPlugin.Step stepId, int frameIndex,
            out OVRPlugin.BodyStateInternal bodyState)
        {
            bodyState = new OVRPlugin.BodyStateInternal()
            {
                // if greater than 0.5 change to posef values
                Confidence = _state.BodyStateConfidence,
                IsActive = _state.BodyStateIsActive,
                JointLocation_0 = new OVRPlugin.BodyJointLocation()
                {
                    Pose = new OVRPlugin.Posef()
                    {
                        Orientation = FakeOVRPluginState.TestRot,
                        Position = FakeOVRPluginState.TestPos,
                    },
                    LocationFlags = _state.LocFlags
                }
            };

            return Result.Success;
        }

        public override Result ovrp_GetBodyTrackingSupported(out Bool value)
        {
            _state.BodyTrackingCount++;
            value = Bool.True;
            return Result.Success;
        }

        public override Result ovrp_GetFaceTrackingSupported(out Bool value)
        {
            _state.FaceTrackingCount++;
            value = Bool.True;
            return Result.Success;
        }

        public override Result ovrp_GetEyeTrackingSupported(out Bool value)
        {
            _state.EyeTrackingCount++;
            value = Bool.True;
            return Result.Success;
        }
    }
#endregion
}

#endif //OVRPLUGIN_TESTING
