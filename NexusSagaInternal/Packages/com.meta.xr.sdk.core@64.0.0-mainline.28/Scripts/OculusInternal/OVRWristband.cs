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

/// <summary>
/// Provides state and events for EMG wristbands.
/// </summary>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_wristband")]
public class OVRWristband : MonoBehaviour
{
    private static OVRPlugin.WristbandState currState;
    private static OVRPlugin.WristbandState prevState;

    /// <summary>
    /// Provides the current state of all connected wristbands.
    /// </summary>
    public static OVRPlugin.WristbandState State
    {
        get { return currState; }
    }

    /// <summary>
    /// Describes the new state of a gesture after an event occurs.
    /// </summary>
    public struct GestureState
    {
        public GestureState(OVRPlugin.WristGesture gesture, OVRPlugin.Handedness handedness, bool pressed)
        {
            this.Gesture = gesture;
            this.Handedness = handedness;
            this.Pressed = pressed;
        }

        public OVRPlugin.WristGesture Gesture;
        public OVRPlugin.Handedness Handedness;
        public bool Pressed;
    }

    public delegate void GestureAction(GestureState state);

    /// <summary>
    /// Occurs when any gesture is pressed or released.
    /// </summary>
    public static event GestureAction OnGesture;

    /// <summary>
    /// Occurs when an index finger pinch gesture is pressed or released.
    /// </summary>
    public static event GestureAction OnPinchIndex;

    /// <summary>
    /// Occurs when a middle finger pinch gesture is pressed or released.
    /// </summary>
    public static event GestureAction OnPinchMiddle;

    /// <summary>
    /// Occurs when a thumb click gesture is pressed or released.
    /// </summary>
    public static event GestureAction OnThumbClick;

    /// <summary>
    /// Occurs when a d-pad up gesture is pressed or released.
    /// </summary>
    public static event GestureAction OnDPadUp;

    /// <summary>
    /// Occurs when a d-pad down gesture is pressed or released.
    /// </summary>
    public static event GestureAction OnDPadDown;

    /// <summary>
    /// Occurs when a d-pad left gesture is pressed or released.
    /// </summary>
    public static event GestureAction OnDPadLeft;

    /// <summary>
    /// Occurs when a d-pad right gesture is pressed or released.
    /// </summary>
    public static event GestureAction OnDPadRight;

    public void Update()
    {
        prevState = currState;
        currState = OVRPlugin.GetWristbandState();

        DispatchGestureEvents(OVRPlugin.WristGesture.PinchIndex);
        DispatchGestureEvents(OVRPlugin.WristGesture.PinchMiddle);
        DispatchGestureEvents(OVRPlugin.WristGesture.ThumbClick);
        DispatchGestureEvents(OVRPlugin.WristGesture.DPadUp);
        DispatchGestureEvents(OVRPlugin.WristGesture.DPadDown);
        DispatchGestureEvents(OVRPlugin.WristGesture.DPadLeft);
        DispatchGestureEvents(OVRPlugin.WristGesture.DPadRight);
    }

    private static void DispatchGestureEvents(OVRPlugin.WristGesture gesture)
    {
        DispatchGestureEvent(gesture, OVRPlugin.Handedness.LeftHanded);
        DispatchGestureEvent(gesture, OVRPlugin.Handedness.RightHanded);
    }

    private static void DispatchGestureEvent(OVRPlugin.WristGesture gesture, OVRPlugin.Handedness handedness)
    {
        bool currPressState = IsGesturePressed(currState, gesture, handedness);
        bool prevPressState = IsGesturePressed(prevState, gesture, handedness);

        if (currPressState != prevPressState)
        {
            GestureState state = new GestureState(gesture, handedness, currPressState);
            GestureAction gestureAction = GetGestureAction(gesture);

            // Dispatch generic `OnGesture` action, guarding against exceptions in user code.
            try
            {
                OnGesture?.Invoke(state);
            }
            catch (System.Exception e)
            {
                Debug.LogException(e);
            }

            // Dispatch specific action for the current gesture type, guarding against exceptions in user code.
            try
            {
                gestureAction?.Invoke(state);
            }
            catch (System.Exception e)
            {
                Debug.LogException(e);
            }
        }
    }

    private static GestureAction GetGestureAction(OVRPlugin.WristGesture gesture)
    {
        switch (gesture)
        {
            case OVRPlugin.WristGesture.PinchIndex:
                return OnPinchIndex;
            case OVRPlugin.WristGesture.PinchMiddle:
                return OnPinchMiddle;
            case OVRPlugin.WristGesture.ThumbClick:
                return OnThumbClick;
            case OVRPlugin.WristGesture.DPadUp:
                return OnDPadUp;
            case OVRPlugin.WristGesture.DPadDown:
                return OnDPadDown;
            case OVRPlugin.WristGesture.DPadLeft:
                return OnDPadLeft;
            case OVRPlugin.WristGesture.DPadRight:
                return OnDPadRight;
            default:
                return null;
        }
    }

    private static bool IsGesturePressed(OVRPlugin.WristbandState state, OVRPlugin.WristGesture gesture,
        OVRPlugin.Handedness handedness)
    {
        return (GetWristGestures(state, handedness) & (uint)gesture) != 0;
    }

    private static uint GetWristGestures(OVRPlugin.WristbandState state, OVRPlugin.Handedness handedness)
    {
        switch (handedness)
        {
            case OVRPlugin.Handedness.LeftHanded:
                return state.LWristGestures;
            case OVRPlugin.Handedness.RightHanded:
                return state.RWristGestures;
            default:
                return 0;
        }
    }
}

#endif // OVR_INTERNAL_CODE
