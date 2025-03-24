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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Direction = OVRMicrogestureNavigationalInput.Direction;

/// <summary>
/// Sample script which reads hand microgesture event data and displays recognized gestures on a UI panel.
/// </summary>
public class OVRMicrogesturesSample : MonoBehaviour
{
    [SerializeField]
    private OVRMicrogestureNavigationalInput microgestureInput;

    [SerializeField]
    private OVRMicrogestureEventSource leftGestureSource;

    [SerializeField]
    private OVRMicrogestureEventSource rightGestureSource;

    [SerializeField]
    private OVRMicrogestureGatingGestureExample leftGate;

    [SerializeField]
    private OVRMicrogestureGatingGestureExample rightGate;

    [Header("Gesture Labels")]
    [SerializeField]
    private Text leftGestureLabel;

    [SerializeField]
    private Text rightGestureLabel;

    [SerializeField]
    private float gestureShowDuration = 1.5f;

    [Header("Gate Labels")]
    [SerializeField]
    private Text leftGateLabel;

    [SerializeField]
    private Text rightGateLabel;

    [Header("Navigation Icons Left")]
    [SerializeField]
    private Image leftArrowL;

    [SerializeField]
    private Image rightArrowL;

    [SerializeField]
    private Image upArrowL;

    [SerializeField]
    private Image downArrowL;

    [SerializeField]
    private Image selectIconL;

    [SerializeField]
    private Image pinchIconL;

    [Header("Navigation Icons Right")]
    [SerializeField]
    private Image leftArrowR;

    [SerializeField]
    private Image rightArrowR;

    [SerializeField]
    private Image upArrowR;

    [SerializeField]
    private Image downArrowR;

    [SerializeField]
    private Image selectIconR;

    [SerializeField]
    private Image pinchIconR;

    [SerializeField]
    private Color initialColor = Color.white;

    [SerializeField]
    private Color highlightColor = Color.blue;

    [SerializeField]
    private float highlightDuration = 1f;

    [Header("Activation")]
    [SerializeField]
    private Text mgActiveLabel;

    [SerializeField]
    private Text gatingActiveLabel;

    [SerializeField]
    private OVRHand leftHand;

    [SerializeField]
    private OVRHand rightHand;

    private bool mgActive;
    private bool gatingActive;
    private bool wasLeftMiddlePinching;
    private bool wasLeftRingPinching;

    private Dictionary<GameObject, Coroutine> highlightCoroutines = new Dictionary<GameObject, Coroutine>();

    void Start()
    {
        microgestureInput.MicrogestureRecognized += OnGestureRecognized;
        microgestureInput.PinchRecognized += OnPinchRecognized;
        microgestureInput.NavigationDirectionRecognized += OnNavDirectionRecognized;
        microgestureInput.NavigationSelected += OnNavSelected;
        microgestureInput.NavigationPinchSelected += OnPinchSelected;

        leftGate.GatingStateChanged += (state) => OnGateStateChanged(OVRPlugin.Hand.HandLeft, state);
        rightGate.GatingStateChanged += (state) => OnGateStateChanged(OVRPlugin.Hand.HandRight, state);

        StartMicrogestureDetection();
    }

    private void Update()
    {
        bool isLeftPinching = leftHand.GetFingerIsPinching(OVRHand.HandFinger.Middle);
        if (isLeftPinching && !wasLeftMiddlePinching)
        {
            ToggleMgActive();
        }
        wasLeftMiddlePinching = isLeftPinching;

        isLeftPinching = leftHand.GetFingerIsPinching(OVRHand.HandFinger.Ring);
        if (isLeftPinching && !wasLeftRingPinching)
        {
            ToggleGatingActive();
        }
        wasLeftRingPinching = isLeftPinching;
    }

    private void OnNavSelected(OVRPlugin.Hand hand)
    {
        switch (hand)
        {
            case OVRPlugin.Hand.HandLeft:
                HighlightIcon(selectIconL);
                break;
            case OVRPlugin.Hand.HandRight:
                HighlightIcon(selectIconR);
                break;
        }
    }

    private void OnPinchSelected(OVRPlugin.Hand hand, bool state)
    {
        switch (hand)
        {
            case OVRPlugin.Hand.HandLeft:
                HighlightIcon(pinchIconL, state);
                break;
            case OVRPlugin.Hand.HandRight:
                HighlightIcon(pinchIconR, state);
                break;
        }
    }

    private void OnNavDirectionRecognized(OVRPlugin.Hand hand, Direction dir)
    {
        switch (dir)
        {
            case Direction.Left:
                HighlightIcon(hand == OVRPlugin.Hand.HandLeft ? leftArrowL : leftArrowR);
                break;
            case Direction.Right:
                HighlightIcon(hand == OVRPlugin.Hand.HandLeft ? rightArrowL : rightArrowR);
                break;
            case Direction.Up:
                HighlightIcon(hand == OVRPlugin.Hand.HandLeft ? upArrowL : upArrowR);
                break;
            case Direction.Down:
                HighlightIcon(hand == OVRPlugin.Hand.HandLeft ? downArrowL : downArrowR);
                break;
        }
    }

    private void HighlightIcon(Image icon)
    {
        if (highlightCoroutines.TryGetValue(icon.gameObject, out Coroutine highlightNavCoroutine))
        {
            StopCoroutine(highlightNavCoroutine);
            highlightCoroutines.Remove(icon.gameObject);
        }
        highlightCoroutines.Add(icon.gameObject, StartCoroutine(HighlightIconCoroutine(icon)));
    }

    private IEnumerator HighlightIconCoroutine(Image navIcon)
    {
        Color initialCol = initialColor;
        navIcon.color = highlightColor;
        float timer = 0;
        while (timer < highlightDuration)
        {
            navIcon.color = Color.Lerp(highlightColor, initialCol, (timer / highlightDuration));
            timer += Time.deltaTime;
            yield return null;
        }
    }

    private void HighlightIcon(Image navIcon, bool state)
    {
        navIcon.color = state ? highlightColor : initialColor;
    }

    private void OnGestureRecognized(OVRPlugin.Hand hand, OVRHand.MicrogestureType gesture)
    {
        switch (hand)
        {
            case OVRPlugin.Hand.HandLeft:
                if (leftGate.IsGatingClosed())
                {
                    OnLeftGestureRecognized(gesture);
                }
                break;
            case OVRPlugin.Hand.HandRight:
                if (rightGate.IsGatingClosed())
                {
                    OnRightGestureRecognized(gesture);
                }
                break;
        }
    }

    private void OnPinchRecognized(OVRPlugin.Hand hand, bool state)
    {
        switch (hand)
        {
            case OVRPlugin.Hand.HandLeft:
                if (leftGate.IsGatingOpen())
                {
                    ShowRecognizedGestureLabel(leftGestureLabel, state ? "Pinch" : "-");
                }
                break;
            case OVRPlugin.Hand.HandRight:
                if (rightGate.IsGatingOpen())
                {
                    ShowRecognizedGestureLabel(rightGestureLabel, state ? "Pinch" : "-");
                }
                break;
        }
    }

    private void OnGateStateChanged(OVRPlugin.Hand hand, OVRMicrogestureGatingGestureExample.GatingState state)
    {
        switch (hand)
        {
            case OVRPlugin.Hand.HandLeft:
                ShowGateLabel(leftGateLabel, state);
                break;
            case OVRPlugin.Hand.HandRight:
                ShowGateLabel(rightGateLabel, state);
                break;
        }
    }


    private void OnLeftGestureRecognized(OVRHand.MicrogestureType gesture)
    {
        ShowRecognizedGestureLabel(leftGestureLabel, gesture.ToString());
    }

    private void OnRightGestureRecognized(OVRHand.MicrogestureType gesture)
    {
        ShowRecognizedGestureLabel(rightGestureLabel, gesture.ToString());
    }

    private void ShowRecognizedGestureLabel(Text gestureLabel, string label)
    {
        if (highlightCoroutines.TryGetValue(gestureLabel.gameObject, out Coroutine showGestureLabelCoroutine))
        {
            StopCoroutine(showGestureLabelCoroutine);
            highlightCoroutines.Remove(gestureLabel.gameObject);
        }
        highlightCoroutines.Add(gestureLabel.gameObject, StartCoroutine(ShowGestureLabel(gestureLabel, label)));
    }

    private IEnumerator ShowGestureLabel(Text gestureLabel, string label)
    {
        gestureLabel.text = label;
        yield return new WaitForSeconds(gestureShowDuration);
        gestureLabel.text = string.Empty;
    }

    private void ShowGateLabel(Text gateLabel, OVRMicrogestureGatingGestureExample.GatingState state)
    {
        switch (state)
        {
            case OVRMicrogestureGatingGestureExample.GatingState.GateOpen:
                gateLabel.text ="Pinch";
                break;
            case OVRMicrogestureGatingGestureExample.GatingState.GateClosed:
                gateLabel.text = "Microgestures";
                break;
            case OVRMicrogestureGatingGestureExample.GatingState.Disabled:
                gateLabel.text = "-";
                break;
        }
    }

    private void StartMicrogestureDetection()
    {
        bool result = OVRPlugin.StartMicrogestureDetection();
        if (result)
        {
            mgActive = true;
        }
        mgActiveLabel.text = mgActive ? "Active" : "Inactive";
    }

    private void StopMicrogestureDetection()
    {
        bool result = OVRPlugin.StopMicrogestureDetection();
        if (result)
        {
            mgActive = false;
        }
        mgActiveLabel.text = mgActive ? "Active" : "Inactive";
    }

    public void ToggleMgActive()
    {
        bool activeState = mgActive;
        if (activeState)
        {
            StopMicrogestureDetection();
        }
        else
        {
            StartMicrogestureDetection();
        }
    }

    public void ToggleGatingActive()
    {
        gatingActive = !gatingActive;
        leftGate.SetGatingActive(gatingActive);
        rightGate.SetGatingActive(gatingActive);

        gatingActiveLabel.text = gatingActive ? "Active" : "Inactive";
        leftGateLabel.text = "-";
        rightGateLabel.text = "-";
    }
}
#endif
