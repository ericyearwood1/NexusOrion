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

using NUnit.Framework;
using System;
using System.Collections;
using System.IO;
using System.Reflection;
using System.Runtime.Serialization.Formatters.Binary;
using UnityEngine;
using UnityEngine.TestTools;

public class FaceExpressionsTests : OVRPluginPlayModeTest
{
    private FakeOVRPlugin_78 _fakeOVRPlugin78;
    private FakeOVRPlugin_92 _fakeOVRPlugin92;
    private FakeOVRPluginConstants _fakeConstants;

    private OVRFaceExpressions _faceExpressions;

    private OVRFaceExpressions FaceExpressions
    {
        get
        {
            if (!_faceExpressions)
            {
                _faceExpressions = GameObject.FindAnyObjectByType<OVRFaceExpressions>();
                Assert.IsNotNull(_faceExpressions);
            }

            return _faceExpressions;
        }
    }

    private IEnumerator CheckFaceTrackingStarted()
    {
        if (!FaceExpressions.gameObject.activeSelf)
        {
            FaceExpressions.gameObject.SetActive(true);
            //Wait one update is called
            yield return null;
        }

        // OnEnable() StartTracking is called
        Assert.IsTrue(FaceExpressions.gameObject.activeSelf);
        Assert.IsTrue(FaceExpressions.FaceTrackingEnabled);
        yield return null;
    }

    private IEnumerator CheckFaceTrackingStopped()
    {
        // OnDisable()->StopTracking is called
        _faceExpressions.gameObject.SetActive(false);
        //Wait one update is called
        yield return null;
        Assert.IsFalse(FaceExpressions.gameObject.activeSelf);
        Assert.IsFalse(FaceExpressions.FaceTrackingEnabled);
        yield return null;
    }

#region UNITY TESTS

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        _fakeConstants = new FakeOVRPluginConstants();
        _fakeOVRPlugin78 = new FakeOVRPlugin_78(_fakeConstants);
        OVRPlugin.OVRP_1_78_0.mockObj = _fakeOVRPlugin78;
        _fakeOVRPlugin92 = new FakeOVRPlugin_92(_fakeConstants);
        OVRPlugin.OVRP_1_92_0.mockObj = _fakeOVRPlugin92;


        GameObject FaceTrackingGO = new GameObject();
        _faceExpressions = FaceTrackingGO.AddComponent<OVRFaceExpressions>();
        //Wait at least one update is called
        yield return null;
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        GameObject.Destroy(_faceExpressions.gameObject);
        yield return null;

        OVRPlugin.OVRP_1_78_0.mockObj = new OVRPlugin.OVRP_1_78_0_TEST();
        OVRPlugin.OVRP_1_92_0.mockObj = new OVRPlugin.OVRP_1_92_0_TEST();
        yield return base.UnityTearDown();
    }

    /// <summary>
    /// Checks if the tracking is started properly
    /// </summary>
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestStartFaceTracking()
    {
        yield return CheckFaceTrackingStarted();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestStartFaceTracking_Fail()
    {
        _faceExpressions.gameObject.SetActive(false);
        _fakeConstants._forceStartTrackingToFail = OVRPlugin.Bool.True;
        yield return null;

        FaceExpressions.gameObject.SetActive(true);
        yield return null;

        Assert.IsFalse(FaceExpressions.FaceTrackingEnabled);
        _fakeConstants._forceStartTrackingToFail = OVRPlugin.Bool.False;
    }

    /// <summary>
    /// Checks if the tracking is stopped properly
    /// </summary>
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestStopTracking()
    {
        yield return CheckFaceTrackingStarted();
        yield return CheckFaceTrackingStopped();
    }

    /// <summary>
    /// Checks if the tracking is stopped properly
    /// </summary>
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestRestartTracking()
    {
        yield return CheckFaceTrackingStarted();
        yield return CheckFaceTrackingStopped();
        yield return CheckFaceTrackingStarted();
    }

    /// <summary>
    /// Checks that the weight of each expression match the weights of the fake expression created when ovrp_GetFaceState() is called
    /// </summary>
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestGetAllFaceExpressionValues()
    {
        bool result = false;
        foreach (OVRFaceExpressions.FaceExpression exp in Enum.GetValues(typeof(OVRFaceExpressions.FaceExpression)))
        {
            if (exp == OVRFaceExpressions.FaceExpression.Max || exp == OVRFaceExpressions.FaceExpression.Invalid)
            {
                continue;
            }

            result = _faceExpressions.TryGetFaceExpressionWeight(exp, out var weight);
            Assert.IsTrue(result);
            Assert.AreEqual(_fakeConstants._fakeWeight, weight);
        }

        yield return null;
    }


    /// <summary>
    /// Checks that the confidence of each region match the confidence of the fake expression created when ovrp_GetFaceState() is called
    /// </summary>
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestGetAllFaceExpressionConfidenceValues()
    {
        bool result = false;
        foreach (OVRFaceExpressions.FaceRegionConfidence reg in Enum.GetValues(
                     typeof(OVRFaceExpressions.FaceRegionConfidence)))
        {
            if (reg == OVRFaceExpressions.FaceRegionConfidence.Max)
            {
                continue;
            }

            result = _faceExpressions.TryGetWeightConfidence(reg, out var confidence);
            Assert.IsTrue(result);
            Assert.AreEqual(_fakeConstants._fakeWeight, confidence);
        }

        yield return null;
    }


    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestFaceTackingPermission()
    {
        yield return CheckFaceTrackingStopped();
        // Set the facetracking permission to false
        OVRPermissionsRequester.fakeIsPermissionGranted = (Permission) => { return false; };

        yield return null;
        FaceExpressions.gameObject.SetActive(true);
        //Wait one update is called
        yield return null;

        Assert.IsFalse(_faceExpressions.enabled);
        OVRPermissionsRequester.fakeIsPermissionGranted = null;
        yield return null;
    }


    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestEyeFollowingBlendshapesValid()
    {
        Assert.IsTrue(_faceExpressions.EyeFollowingBlendshapesValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestCheckValidy_FailAndRaiseException()
    {
        _fakeConstants._forceGetFaceStateToFail = OVRPlugin.Bool.True;
        yield return null;
        Assert.Throws<InvalidOperationException>(() => { _faceExpressions.CheckValidity(); });
        _fakeConstants._forceGetFaceStateToFail = OVRPlugin.Bool.False;
        yield return null;
    }


    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestGetFaceExpressionWeight_Invalid()
    {
        OVRFaceExpressions.FaceExpression invalidExpression =
            (OVRFaceExpressions.FaceExpression)((int)OVRFaceExpressions.FaceExpression.Max + 1);
        Assert.IsFalse(_faceExpressions.TryGetFaceExpressionWeight(invalidExpression, out var weigth));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestGetWeightConfidence_Invalid()
    {
        OVRFaceExpressions.FaceRegionConfidence invalidRegion =
            (OVRFaceExpressions.FaceRegionConfidence)((int)OVRFaceExpressions.FaceRegionConfidence.Max + 1);
        Assert.IsFalse(_faceExpressions.TryGetWeightConfidence(invalidRegion, out var weigth));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestSquareBracketOperatorOverload_Valid()
    {
        Assert.AreEqual(_fakeConstants._fakeWeight,
            _faceExpressions[OVRFaceExpressions.FaceExpression.JawDrop]);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestSquareBracketOperatorOverload_Invalid()
    {
        OVRFaceExpressions.FaceExpression invalidExpression =
            (OVRFaceExpressions.FaceExpression)((int)OVRFaceExpressions.FaceExpression.Max + 1);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
        {
            var result = _faceExpressions[invalidExpression];
        });
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestToArray()
    {
        var faceExpressionsArray = _faceExpressions.ToArray();
        foreach (var weight in faceExpressionsArray)
        {
            Assert.AreEqual(_fakeConstants._fakeWeight, weight);
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestSetDataSources()
    {
        yield return CheckFaceTrackingStopped();
        var testDataSources = new OVRPlugin.FaceTrackingDataSource[] { OVRPlugin.FaceTrackingDataSource.Visual };
        OVRRuntimeSettings.Instance.RequestsVisualFaceTracking = true;
        OVRRuntimeSettings.Instance.RequestsAudioFaceTracking = false;
        yield return CheckFaceTrackingStarted();
        Assert.AreEqual(testDataSources, _fakeOVRPlugin92.RequestedDataSources);

        yield return CheckFaceTrackingStopped();
        testDataSources = new OVRPlugin.FaceTrackingDataSource[] { OVRPlugin.FaceTrackingDataSource.Visual, OVRPlugin.FaceTrackingDataSource.Audio };
        OVRRuntimeSettings.Instance.RequestsVisualFaceTracking = true;
        OVRRuntimeSettings.Instance.RequestsAudioFaceTracking = true;
        yield return CheckFaceTrackingStarted();
        CollectionAssert.AreEquivalent(testDataSources, _fakeOVRPlugin92.RequestedDataSources);

        yield return CheckFaceTrackingStopped();
        testDataSources = new OVRPlugin.FaceTrackingDataSource[] { OVRPlugin.FaceTrackingDataSource.Audio };
        OVRRuntimeSettings.Instance.RequestsVisualFaceTracking = false;
        OVRRuntimeSettings.Instance.RequestsAudioFaceTracking = true;
        yield return CheckFaceTrackingStarted();
        Assert.AreEqual(testDataSources, _fakeOVRPlugin92.RequestedDataSources);
    }

#endregion
}

#endif
