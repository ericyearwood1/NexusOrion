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
using System.Reflection;

public class FakeOVRPluginConstants
{
    public Dictionary<string, float> _fakeSpecificWeights = new Dictionary<string, float>();
    public float _fakeWeight = 0.5f;
    public OVRPlugin.Bool _forceStartTrackingToFail = OVRPlugin.Bool.False;
    public OVRPlugin.Bool _forceGetFaceStateToFail = OVRPlugin.Bool.False;

    private const string fieldPrefixExpressionWeights = "ExpressionWeights_";

    public void PopulateFaceExpressionWeights<T>(ref T FaceState)
    {
        // FaceState has to be converted to an object, so its fields can be modified
        object FaceSateObject = FaceState;

        FieldInfo[] fields = FaceSateObject.GetType().GetFields();
        for (int i = 0; i < fields.Length; i++)
        {
            if (fields[i].Name.Contains(fieldPrefixExpressionWeights))
            {
                var suffix = fields[i].Name.Substring(fieldPrefixExpressionWeights.Length);
                float val = _fakeSpecificWeights.ContainsKey(suffix) ? _fakeSpecificWeights[suffix] : _fakeWeight;
                fields[i].SetValue(FaceSateObject, val);
            }
            else if (fields[i].Name.Contains("ExpressionWeightConfidences_"))
            {
                fields[i].SetValue(FaceSateObject, _fakeWeight);
            }
            else if (fields[i].Name == "Status")
            {
                fields[i].SetValue(FaceSateObject, new OVRPlugin.FaceExpressionStatusInternal()
                {
                    IsValid = OVRPlugin.Bool.True,
                    IsEyeFollowingBlendshapesValid = OVRPlugin.Bool.True,
                });
            }
            else if (fields[i].Name == "Time")
            {
                fields[i].SetValue(FaceSateObject, 0.0f);
            }
        }

        FaceState = (T)FaceSateObject;
    }
}

public class FakeOVRPlugin_78 : OVRPlugin.OVRP_1_78_0_TEST
{
    public FakeOVRPluginConstants _fakeConstants;
    public OVRPlugin.FaceStateInternal _fakeExpression = new OVRPlugin.FaceStateInternal();

    public FakeOVRPlugin_78(FakeOVRPluginConstants fakeConstant)
    {
        _fakeConstants = fakeConstant;
        _fakeConstants.PopulateFaceExpressionWeights<OVRPlugin.FaceStateInternal>(ref _fakeExpression);
    }

    public OVRPlugin.Bool _fakeFaceTrackingEnabled = OVRPlugin.Bool.False;

#region MOCKED METHODS

    public override OVRPlugin.Result ovrp_StartFaceTracking()
    {
        _fakeFaceTrackingEnabled = OVRPlugin.Bool.True;
        return _fakeConstants._forceStartTrackingToFail == OVRPlugin.Bool.False
            ? OVRPlugin.Result.Success
            : OVRPlugin.Result.Failure;
    }

    public override OVRPlugin.Result ovrp_StopFaceTracking()
    {
        _fakeFaceTrackingEnabled = OVRPlugin.Bool.False;
        return OVRPlugin.Result.Success;
    }

    public override OVRPlugin.Result ovrp_GetFaceTrackingEnabled(out OVRPlugin.Bool faceTrackingEnabled)
    {
        faceTrackingEnabled = _fakeFaceTrackingEnabled;
        return OVRPlugin.Result.Success;
    }

    public override OVRPlugin.Result ovrp_GetFaceState(OVRPlugin.Step stepId, int frameIndex,
        out OVRPlugin.FaceStateInternal faceState)
    {
        faceState = _fakeExpression;
        return _fakeConstants._forceGetFaceStateToFail == OVRPlugin.Bool.False
            ? OVRPlugin.Result.Success
            : OVRPlugin.Result.Failure;
    }

#endregion
}

public class FakeOVRPlugin_92 : OVRPlugin.OVRP_1_92_0_TEST
{
    public FakeOVRPluginConstants FakeConstants;

    public OVRPlugin.FaceState2Internal FakeFaceExpression2 = new OVRPlugin.FaceState2Internal();

    public FakeOVRPlugin_92(FakeOVRPluginConstants fakeConstants)
    {
        FakeConstants = fakeConstants;

        FakeConstants.PopulateFaceExpressionWeights<OVRPlugin.FaceState2Internal>(ref FakeFaceExpression2);
    }

    #region MOCKED METHODS
    public OVRPlugin.Bool _fakeFaceTrackingEnabled = OVRPlugin.Bool.False;
    public OVRPlugin.FaceTrackingDataSource[] RequestedDataSources;

    public override OVRPlugin.Result ovrp_GetFaceState2(OVRPlugin.Step stepId, int frameIndex,
        out OVRPlugin.FaceState2Internal faceState)
    {
        faceState = FakeFaceExpression2;
        return FakeConstants._forceGetFaceStateToFail == OVRPlugin.Bool.False
            ? OVRPlugin.Result.Success
            : OVRPlugin.Result.Failure;
    }

    public override OVRPlugin.Result ovrp_GetFaceTracking2Enabled(out OVRPlugin.Bool faceTracking2Enabled)
    {
        faceTracking2Enabled = _fakeFaceTrackingEnabled;
        return OVRPlugin.Result.Success;
    }

    public override OVRPlugin.Result ovrp_StartFaceTracking2(OVRPlugin.FaceTrackingDataSource[] requestedDataSources, uint requestedDataSourcesCount)
    {
        this.RequestedDataSources = (OVRPlugin.FaceTrackingDataSource[])requestedDataSources.Clone();

        _fakeFaceTrackingEnabled = OVRPlugin.Bool.True;
        return FakeConstants._forceStartTrackingToFail == OVRPlugin.Bool.False
            ? OVRPlugin.Result.Success
            : OVRPlugin.Result.Failure;
    }

    public override OVRPlugin.Result ovrp_StopFaceTracking2()
    {
        _fakeFaceTrackingEnabled = OVRPlugin.Bool.False;
        return OVRPlugin.Result.Success;
    }
    #endregion
}

#endif
