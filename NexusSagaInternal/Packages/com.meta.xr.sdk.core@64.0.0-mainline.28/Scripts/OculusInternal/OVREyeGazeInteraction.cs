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

#if OVR_INTERNAL_CODE // OVR_EYE_GAZE_INTERACTION

using System;
using UnityEngine;

/// <summary>
/// This class updates the transform of the GameObject to point toward an eye direction.
/// </summary>
/// <remarks>
/// See <see cref="OVRPlugin.EyeGazeInteractionState"/> structure for list of eye state parameters.
/// </remarks>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_eye_gaze_interaction")]
public class OVREyeGazeInteraction : MonoBehaviour
{
    /// <summary>
    /// True if eye tracking is enabled, otherwise false.
    /// </summary>
    public bool EyeTrackingEnabled => OVRPlugin.eyeTrackingEnabled;

    /// <summary>
    /// GameObject will automatically change position and rotate according to the selected eye.
    /// </summary>
    public EyeId Eye;

    /// <summary>
    /// GameObject will automatically adjust transform to match the eye transform
    /// </summary>
    public bool ApplyTransform = true;

    private OVRPlugin.EyeGazeInteractionsState _currentEyeGazeInteractionsState;

    private Transform _viewTransform;

    private const OVRPermissionsRequester.Permission EyeTrackingPermission =
        OVRPermissionsRequester.Permission.EyeTracking;

    private Action<string> _onPermissionGranted;

    private void Awake()
    {
        _onPermissionGranted = OnPermissionGranted;
    }

    private void Start()
    {
    }

    private void OnEnable()
    {
        if (!StartEyeTracking())
        {
            enabled = false;
        }
    }

    private void OnPermissionGranted(string permissionId)
    {
        Debug.LogWarning("EyeTrack OnPermissionGranted " + permissionId);
        OVRPermissionsRequester.PermissionGranted -= _onPermissionGranted;
        if (!enabled)
        {
            Debug.LogWarning("EyeTrack Trying to start");
            StartEyeTracking();
            enabled = true;
        }
    }

    private bool StartEyeTracking()
    {
        if (!OVRPermissionsRequester.IsPermissionGranted(EyeTrackingPermission))
        {
            Debug.LogWarning("EyeTrack no eye tracking permission");
            OVRPermissionsRequester.PermissionGranted -= _onPermissionGranted;
            OVRPermissionsRequester.PermissionGranted += _onPermissionGranted;
            return false;
        }

        return true;
    }

    private void OnDisable()
    {
    }

    private void OnDestroy()
    {
        OVRPermissionsRequester.PermissionGranted -= _onPermissionGranted;
    }

    private void Update()
    {
        if (!OVRPlugin.GetEyeGazeInteractionsState(OVRPlugin.Step.Render, -1, ref _currentEyeGazeInteractionsState))
        {
            return;
        }

        var eyeGaze = _currentEyeGazeInteractionsState.EyeGazeInteractions[(int)Eye];

        if (!eyeGaze.IsValid)
            return;

        var pose = eyeGaze.Pose.ToOVRPose();

        if (ApplyTransform)
        {
            transform.position = pose.position;
            transform.rotation = pose.orientation;
        }
    }

    /// <summary>
    /// List of eyes
    /// </summary>
    public enum EyeId
    {
        Combined = OVRPlugin.EyeInteraction.Combined,
        Left = OVRPlugin.EyeInteraction.Left,
        Right = OVRPlugin.EyeInteraction.Right
    }
}

#endif
