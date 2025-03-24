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
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MicrogestureType = OVRHand.MicrogestureType;

/// <summary>
/// Emits 2D navigation events that can be used to drive UI input based on OVRMicrogestureHandEventSources.
/// </summary>
public class OVRMicrogestureNavigationalInput : MonoBehaviour
{
    /// <summary>
    /// Enables emulation of gesture navigation events via key presses.
    /// </summary>
    [SerializeField]

    private bool enableDebugKeys = true;

    [Header("Gesture Sources")]
    [SerializeField]
    private OVRMicrogestureEventSource leftGestureSource;

    [SerializeField]
    private OVRMicrogestureEventSource rightGestureSource;

    public enum Direction
    {
        None = 0,
        Left = 1,
        Right = 2,
        Up = 3,
        Down = 4,
    }

    /// <summary>
    /// Action fired which contains the recognized gesture and the hand side.
    /// </summary>
    public Action<OVRPlugin.Hand, MicrogestureType> MicrogestureRecognized { get; set; }

    /// <summary>
    /// Action fired which contains the recognized gesture and the hand side.
    /// </summary>
    public Action<OVRPlugin.Hand, bool> PinchRecognized { get; set; }

    /// <summary>
    /// Action fired which contains a 2D directional navigation and the hand side.
    /// </summary>
    public Action<OVRPlugin.Hand, Direction> NavigationDirectionRecognized { get; set; }

    public Action<OVRPlugin.Hand> NavigationSelected { get; set; }

    public Action<OVRPlugin.Hand, bool> NavigationPinchSelected { get; set; }

    /// <summary>
    /// Maps microgestures to directional navigation events for the left hand.
    /// </summary>
    private Dictionary<MicrogestureType, Direction> leftGestureToDirection = new Dictionary<MicrogestureType, Direction>
    {
        { MicrogestureType.SwipeTip, Direction.Right },
        { MicrogestureType.SwipeKnuckle, Direction.Left },
        { MicrogestureType.SwipeForward, Direction.Up },
        { MicrogestureType.SwipeBackward, Direction.Down },

    };

    /// <summary>
    /// Maps microgestures to directional navigation events for the right hand.
    /// </summary>
    private Dictionary<MicrogestureType, Direction> rightGestureToDirection = new Dictionary<MicrogestureType, Direction>
    {
        { MicrogestureType.SwipeTip, Direction.Left },
        { MicrogestureType.SwipeKnuckle, Direction.Right },
        { MicrogestureType.SwipeForward, Direction.Up },
        { MicrogestureType.SwipeBackward, Direction.Down },

    };

    private void Start()
    {
        leftGestureSource.GestureRecognized += ((gesture) => OnGestureRecognized(OVRPlugin.Hand.HandLeft, gesture));
        rightGestureSource.GestureRecognized += ((gesture) => OnGestureRecognized(OVRPlugin.Hand.HandRight, gesture));
        leftGestureSource.PinchRecognized += (state) => OnPinchRecognized(OVRPlugin.Hand.HandLeft, state);
        rightGestureSource.PinchRecognized += (state) => OnPinchRecognized(OVRPlugin.Hand.HandRight, state);
    }

    private void Update()
    {
        if (enableDebugKeys)
        {
            CheckForGestureKeyPresses();
        }
    }

    private void CheckForGestureKeyPresses()
    {
        if (Input.GetKeyDown(KeyCode.A))           // WASD controls for directional gesture emulation
        {
            OnGestureRecognized(OVRPlugin.Hand.HandRight, MicrogestureType.SwipeTip);
        }
        else if (Input.GetKeyDown(KeyCode.D))
        {
            OnGestureRecognized(OVRPlugin.Hand.HandRight, MicrogestureType.SwipeKnuckle);
        }
        else if (Input.GetKeyDown(KeyCode.W))
        {
            OnGestureRecognized(OVRPlugin.Hand.HandRight, MicrogestureType.SwipeForward);
        }
        else if (Input.GetKeyDown(KeyCode.S))
        {
            OnGestureRecognized(OVRPlugin.Hand.HandRight, MicrogestureType.SwipeBackward);
        }
        else if (Input.GetKeyDown(KeyCode.T))
        {
            OnGestureRecognized(OVRPlugin.Hand.HandRight, MicrogestureType.ThumbTap);
        }
        else if (Input.GetKeyDown(KeyCode.P))
        {
            OnPinchRecognized(OVRPlugin.Hand.HandRight, true);
        }
        else if (Input.GetKeyUp(KeyCode.P))
        {
            OnPinchRecognized(OVRPlugin.Hand.HandRight, false);
        }
    }


    private void ProcessGestureAsNavigationEvent(OVRPlugin.Hand hand, MicrogestureType gestureType)
    {
        Direction direction;
        switch (hand)
        {
            case OVRPlugin.Hand.HandLeft:
                if (leftGestureToDirection.TryGetValue(gestureType, out direction))
                {
                    NavigationDirectionRecognized?.Invoke(hand, direction);
                }
                break;
            case OVRPlugin.Hand.HandRight:
                if (rightGestureToDirection.TryGetValue(gestureType, out direction))
                {
                    NavigationDirectionRecognized?.Invoke(hand, direction);
                }
                break;
        }
    }

    private void OnGestureRecognized(OVRPlugin.Hand hand, MicrogestureType gestureType)
    {
        switch (gestureType)

        {
            case MicrogestureType.SwipeKnuckle:
            case MicrogestureType.SwipeTip:
            case MicrogestureType.SwipeForward:
            case MicrogestureType.SwipeBackward:
                ProcessGestureAsNavigationEvent(hand, gestureType);
                break;

            case MicrogestureType.ThumbTap:
                NavigationSelected?.Invoke(hand);
                break;
        }
        MicrogestureRecognized?.Invoke(hand, gestureType);
    }

    private void OnPinchRecognized(OVRPlugin.Hand hand, bool state)
    {
        NavigationPinchSelected?.Invoke(hand, state);
        PinchRecognized?.Invoke(hand, state);
    }
}
#endif
