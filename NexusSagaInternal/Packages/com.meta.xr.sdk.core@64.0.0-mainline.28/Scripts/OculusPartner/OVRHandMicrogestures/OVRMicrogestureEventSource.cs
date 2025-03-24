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
/// Emits events based on recognized microgestures for the referenced OVRHand.
/// </summary>
public class OVRMicrogestureEventSource : MonoBehaviour
{
    private const float MinTimeBetweenGestures = 0.07f;

    [SerializeField]
    private OVRHand _hand;

    // Microgesture recognized action
    public Action<OVRHand.MicrogestureType> GestureRecognized;

    // Pinch recognized action
    public Action<bool> PinchRecognized;

    private float _lastGestureTime;
    private OVRHand.MicrogestureType _lastDetectedGesture;

    private bool _lastPinchState = false;

    private void Update()
    {
        bool indexPinch = _hand.GetFingerIsPinching(OVRHand.HandFinger.Index);
        if (_lastPinchState != indexPinch)
        {
            PinchRecognized?.Invoke(indexPinch);
        }
        _lastPinchState = indexPinch;

        OVRHand.MicrogestureType mgType = _hand.GetMicrogestureType();
        if (mgType != OVRHand.MicrogestureType.Invalid && mgType != OVRHand.MicrogestureType.NoGesture)
        {
            CheckForNewGesture(mgType);
        }
    }

    private void CheckForNewGesture(OVRHand.MicrogestureType gesture)
    {
        if (_lastDetectedGesture == gesture && Time.time - _lastGestureTime < MinTimeBetweenGestures)
        {
            // Filter out double triggers of the same gesture
            return;
        }
        _lastDetectedGesture = gesture;
        _lastGestureTime = Time.time;
        GestureRecognized?.Invoke(gesture);
    }
}
#endif
