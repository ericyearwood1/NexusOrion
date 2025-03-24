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

using UnityEngine;

/// <summary>
/// A component for automatic plane detection.
/// </summary>
/// <remarks>Add / remove this component in the Unity scene to start / stop the plane detection.</remarks>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_plane_tracker")]
public sealed class OVRPlaneTracker : MonoBehaviour
{
    /// <summary>
    /// Indicates whether the current device supports plane detection.
    /// </summary>
    public bool PlaneDetectionSupported => OVRPlugin.planeTrackingSupported;

    /// <summary>
    /// Indicates whether the plane detection is running.
    /// </summary>
    public bool PlaneDetectionActive => _planeTrackingStatus == PlaneTrackingStatus.Running;

    internal enum PlaneTrackingStatus
    {
        Stopped, // Plane tracker stopped
        Running, // Plane tracker running
        WaitingForResult, // Waiting for start or stop callback result
    }

    internal ulong _planeTrackerRequestId;
    internal PlaneTrackingStatus _planeTrackingStatus = PlaneTrackingStatus.Stopped;

    private void Awake()
    {
        RegisterPlaneTrackerEvents();
    }

    private void Update() => SyncPlaneTrackerState();

    private void SyncPlaneTrackerState()
    {
        if (!PlaneDetectionSupported) return;

        if (enabled)
        {
            if (_planeTrackingStatus == PlaneTrackingStatus.Running ||
                _planeTrackingStatus == PlaneTrackingStatus.WaitingForResult)
                return;

            OVRPlugin.CreatePlaneTracker(new OVRPlugin.PlaneTrackerBaseCreateInfo(), out _planeTrackerRequestId);
            _planeTrackingStatus = PlaneTrackingStatus.WaitingForResult;
            OVRSceneManager.Development.Log(nameof(OVRPlaneTracker),
                $"Starting plane detection. Request id: [{_planeTrackerRequestId}]");
        }
        else
        {
            if (_planeTrackingStatus != PlaneTrackingStatus.Running) return;

            OVRPlugin.DestroyPlaneTracker(out _planeTrackerRequestId);
            _planeTrackingStatus = PlaneTrackingStatus.WaitingForResult;
            OVRSceneManager.Development.Log(nameof(OVRPlaneTracker), "Stopping plane detection.");
        }
    }

    private void OnCreatePlaneTrackerResults(ulong requestId, bool result)
    {
        if (_planeTrackerRequestId != requestId)
        {
            OVRSceneManager.Development.LogWarning(nameof(OVRPlaneTracker),
                $"[{nameof(OnCreatePlaneTrackerResults)}] " +
                $"Mismatched request ids [{_planeTrackerRequestId}] [{requestId}]");
            return;
        }

        if (result)
        {
            _planeTrackingStatus = PlaneTrackingStatus.Running;
            OVRSceneManager.Development.Log(nameof(OVRPlaneTracker),
                $"[{nameof(OnCreatePlaneTrackerResults)}] Plane detection started.");
        }
        else
        {
            _planeTrackingStatus = PlaneTrackingStatus.Stopped;
            OVRSceneManager.Development.LogWarning(nameof(OVRPlaneTracker),
                $"[{nameof(OnCreatePlaneTrackerResults)}] Plane tracking creation failed.");
        }
    }

    private void OnDestroyPlaneTrackerResults(ulong requestId, bool result)
    {
        if (requestId != _planeTrackerRequestId) return;
        if (result)
        {
            _planeTrackingStatus = PlaneTrackingStatus.Stopped;
        }
        else
        {
            _planeTrackingStatus = PlaneTrackingStatus.Running;
            OVRSceneManager.Development.LogWarning(nameof(OVRPlaneTracker), $"Failed to stop plane tracking.");
        }
    }

    private void OnDisable()
    {
        SyncPlaneTrackerState();
    }

    private void OnDestroy()
    {
        _planeTrackingStatus = PlaneTrackingStatus.Stopped;
        UnregisterPlaneTrackerEvents();
    }

    private void RegisterPlaneTrackerEvents()
    {
        OVRManager.CreatePlaneTrackerResults += OnCreatePlaneTrackerResults;
        OVRManager.DestroyPlaneTrackerResults += OnDestroyPlaneTrackerResults;
    }

    private void UnregisterPlaneTrackerEvents()
    {
        OVRManager.CreatePlaneTrackerResults -= OnCreatePlaneTrackerResults;
        OVRManager.DestroyPlaneTrackerResults -= OnDestroyPlaneTrackerResults;
    }
}
#endif // OVR_INTERNAL_CODE
