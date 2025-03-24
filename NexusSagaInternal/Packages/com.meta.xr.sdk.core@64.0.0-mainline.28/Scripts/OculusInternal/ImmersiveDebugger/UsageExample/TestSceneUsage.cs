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

#if OVR_INTERNAL_CODE

using Meta.XR.ImmersiveDebugger;
using Meta.XR.ImmersiveDebugger.Gizmo;
using System;
using UnityEngine;

/// <summary>
/// This is a usage demonstration of the Immersive Debugging Framework,
/// illustrating how to use OVRDebug attributes to display
/// runtime control panel and gizmos in existing scene.
///
/// Used by Oculus/VR/Scenes/OculusInternal/ImmersiveDebuggerTest scene.
///
/// </summary>
[RequireComponent(typeof(OVREyeGaze))]
internal class TestSceneUsage : MonoBehaviour
{
    private OVREyeGaze _eyeGazeComponent;

    [DebugMember(GizmoType = DebugGizmoType.Axis)]
    private Pose _eyeGazePose;

    [DebugMember(GizmoType = DebugGizmoType.Point)]
    private Vector3 _eyeGazePosition;

    [DebugMember(DebugColor.Gray, GizmoType = DebugGizmoType.Line)]
    private Tuple<Vector3,Vector3> _eyeGazeDirection;

    [DebugMember]
    private float _confidence;

    [DebugMember(Tweakable = true, Min = 0.1f, Max = 1.0f)]
    private float drawingLineWidth = 0.01f;

    // This should be retagged for DebugMember after boolean tweakable supported
    private bool passthroughEnabled = true;
    private bool previousPassthroughEnabled = true;

    private void Start()
    {
        _eyeGazeComponent = GetComponent<OVREyeGaze>();
    }

    private void Update()
    {
        _eyeGazePose.position = transform.position;
        _eyeGazePose.position.z += 0.15f; // explicitly add z offset so gizmo show in front of eye
        _eyeGazePose.rotation = transform.rotation;

        _eyeGazePosition = _eyeGazePose.position;

        var gazeDirEnd = _eyeGazePosition;
        gazeDirEnd += _eyeGazePose.rotation*Vector3.forward * 2;
        _eyeGazeDirection = Tuple.Create(_eyeGazePosition, gazeDirEnd);

        _confidence = _eyeGazeComponent.Confidence;

        DebugGizmos.LineWidth = drawingLineWidth;

        if (passthroughEnabled != previousPassthroughEnabled)
        {
            var rig = FindAnyObjectByType<OVRCameraRig>();
            var manager = rig.GetComponent<OVRManager>();
            manager.isInsightPassthroughEnabled = passthroughEnabled;
            previousPassthroughEnabled = passthroughEnabled;
        }
    }

    [DebugMember]
    private void TogglePassthrough()
    {
        TogglePassthroughStatic();
    }

    [DebugMember]
    private static void TogglePassthroughStatic()
    {
        var rig = FindAnyObjectByType<OVRCameraRig>();
        var manager = rig.GetComponent<OVRManager>();
        manager.isInsightPassthroughEnabled = !manager.isInsightPassthroughEnabled;
    }
}

#endif // OVR_INTERNAL_CODE
