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

using System;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Logs wristband events for testing.
/// </summary>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_wristband_test")]
public class OVRWristbandTest : MonoBehaviour
{
    /// <summary>
    /// Whether gesture events should be logged. Events will be logged to the Unity console by
    //// default, and these will also show up in Android logcat. If `LogDisplay` is not null, the
    /// latest log message will be displayed there as well.
    /// </summary>
    public bool LogEvents = false;

    /// <summary>
    /// If not null, a UI text element to display the latest log message.
    /// </summary>
    public Text LogDisplay;

    void OnEnable()
    {
        Log("[OVRWristbandTest] OnEnable");
        OVRWristband.OnGesture += OnGesture;
    }

    void OnDisable()
    {
        Log("[OVRWristbandTest] OnDisable");
        OVRWristband.OnGesture -= OnGesture;
    }

    private void OnGesture(OVRWristband.GestureState state)
    {
        Log("[OVRWristbandTest] " + getGestureStateString(state));
    }

    private void Log(string msg)
    {
        if (LogEvents)
        {
            Debug.Log(msg);

            if (LogDisplay != null)
            {
                LogDisplay.text = msg;
            }
        }
    }

    String getGestureStateString(OVRWristband.GestureState state)
    {
        return String.Format("gesture: {0}, handedness: {1}, pressed: {2}", state.Gesture, state.Handedness,
            state.Pressed);
    }
}

#endif // OVR_INTERNAL_CODE
