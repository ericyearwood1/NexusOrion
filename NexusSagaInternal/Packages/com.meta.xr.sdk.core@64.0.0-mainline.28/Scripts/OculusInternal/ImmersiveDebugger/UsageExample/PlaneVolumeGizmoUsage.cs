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
using System;
using UnityEngine;

public class PlaneVolumeGizmoUsage : MonoBehaviour
{
    private OVRSceneAnchor _sceneAnchor;
    private OVRScenePlane _scenePlane;
    private OVRSceneVolume _sceneVolume;

    [DebugMember(DebugColor.Red, GizmoType = DebugGizmoType.Plane)]
    private Tuple<Pose, float, float> planeAnchors;

    [DebugMember(DebugColor.Red, GizmoType = DebugGizmoType.TopCenterBox)]
    private Tuple<Pose, float, float, float> volumeAnchors;

    void Start()
    {
        _sceneAnchor = GetComponent<OVRSceneAnchor>();
        _scenePlane = _sceneAnchor.GetComponent<OVRScenePlane>();
        _sceneVolume = _sceneAnchor.GetComponent<OVRSceneVolume>();
    }

    void Update()
    {
        var t = transform;
        if (_scenePlane)
        {
            planeAnchors = new Tuple<Pose, float, float>(new Pose(t.position, t.rotation), _scenePlane.Width, _scenePlane.Height);
        }
        if (_sceneVolume)
        {
            volumeAnchors = new Tuple<Pose, float, float, float>(new Pose(t.position, t.rotation), _sceneVolume.Width, _sceneVolume.Height, _sceneVolume.Depth);
        }
    }
}

#endif // OVR_INTERNAL_CODE
