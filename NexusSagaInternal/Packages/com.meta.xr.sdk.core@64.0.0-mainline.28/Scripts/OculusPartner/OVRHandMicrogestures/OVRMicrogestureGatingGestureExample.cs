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

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
using System;
using UnityEngine;

/// <summary>
/// Example gating gesture: Middle finger curling
/// </summary>
public class OVRMicrogestureGatingGestureExample : MonoBehaviour
{
    [SerializeField]
    private OVRSkeleton _skeleton;

    /// <summary>
    /// State of the date: Disabled, open or closed
    /// </summary>
    public enum GatingState
    {
        Disabled = 0,
        GateOpen = 1,
        GateClosed = 2,
    }

    /// <summary>
    /// Action notifying any change of the state of the gate
    /// </summary>
    public Action<GatingState> GatingStateChanged;

    /// <summary>
    /// Middle finger curl gate lower threshold in degrees
    /// </summary>
    private float _lowMiddleThresholdDegrees = 120.0f;

    /// <summary>
    /// Middle finger curl gate high threshold in degrees
    /// </summary>
    private float _highMiddleThresholdDegrees = 150.0f;

    /// <summary>
    /// Current gating state
    /// </summary>
    private GatingState _gatingState = GatingState.Disabled;

    private void Update()
    {
        // The gate is defined by the total angle formed by all joint of the middle finger
        // - Curled a lot -> Gate Closed state: Imagine you're pressing a trigger with your middle finger
        // - Not curled -> Gate Open state: You released that virtual button
        float angle = GetGatingFingerCurlAngle();
        switch(_gatingState)
        {
            case GatingState.GateOpen:
                if (angle > _highMiddleThresholdDegrees)
                {
                    _gatingState = GatingState.GateClosed;
                    GatingStateChanged?.Invoke(_gatingState);
                }
                break;
            case GatingState.GateClosed:
                if (angle < _lowMiddleThresholdDegrees)
                {
                    _gatingState = GatingState.GateOpen;
                    GatingStateChanged?.Invoke(_gatingState);
                }
                break;
            case GatingState.Disabled:
                break;
        }
    }

    private float GetGatingFingerCurlAngle()
    {
        if (_skeleton.Bones.Count > 0)
        {
            // Compute the total angle formed by all joint of the middle finger
            Quaternion q0 = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.rotation;
            Quaternion q1 = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_Middle1].Transform.rotation;
            Quaternion q2 = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_Middle2].Transform.rotation;
            Quaternion q3 = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_Middle3].Transform.rotation;
            float angle = Quaternion.Angle(q0, q1) + Quaternion.Angle(q1, q2) + Quaternion.Angle(q2, q3);
            return angle;
        } else
        {
            return 0;
        }
    }

    public void SetGatingActive(bool active)
    {
        if (active && _gatingState == GatingState.Disabled)
        {
            // Set the initial state
            float angle = GetGatingFingerCurlAngle();
            if (angle > (_lowMiddleThresholdDegrees + _highMiddleThresholdDegrees) / 2.0)
            {
                _gatingState = GatingState.GateClosed;
            }
            else
            {
                _gatingState = GatingState.GateOpen;
            }
            GatingStateChanged?.Invoke(_gatingState);
        }
        else if (!active && _gatingState != GatingState.Disabled)
        {
            // Deactivate the gating
            _gatingState = GatingState.Disabled;
            GatingStateChanged?.Invoke(_gatingState);
        }
    }

    public bool IsGatingOpen()
    {
        return _gatingState == GatingState.Disabled || _gatingState == GatingState.GateOpen;
    }

    public bool IsGatingClosed()
    {
        return _gatingState == GatingState.Disabled || _gatingState == GatingState.GateClosed;
    }

}
#endif
