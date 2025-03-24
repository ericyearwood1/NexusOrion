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

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;
using UnityEngine.UIElements;
using Button = UnityEngine.UI.Button;

public class OVRVirtualKeyboardSamplePoseControllerInternal : MonoBehaviour
{
    public enum PoseControlValueTypes
    {
        Position,
        Rotation,
        Scale,
    }

    [System.Serializable]
    public struct PoseControls
    {
        public Button PlusButton;
        public Button MinusButton;
        public Text Value;
        public PoseControlValueTypes ValueType;
        public int ValueIndex;
        public float StepSize;
        public string Formatting;

        public void UpdateText(Transform keyboardTransform)
        {
            GetTransformDetails(keyboardTransform, out var kbPosition, out var rotEuler, out var kbScale);
            float value;
            switch (ValueType)
            {
                case PoseControlValueTypes.Position:
                    value = kbPosition[ValueIndex];
                    break;
                case PoseControlValueTypes.Rotation:
                    value = rotEuler[ValueIndex];
                    break;
                case PoseControlValueTypes.Scale:
                    value = kbScale;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
            Value.text = value.ToString(Formatting);
        }

        public void ModifyTransform(Transform keyboardTransform, bool decreaseValue = false)
        {
            GetTransformDetails(keyboardTransform, out var kbPosition, out var rotEuler, out var kbScale);
            var increment = (decreaseValue ? -1 : 1) * StepSize;
            switch (ValueType)
            {
                case PoseControlValueTypes.Position:
                    kbPosition[ValueIndex] += increment;
                    break;
                case PoseControlValueTypes.Rotation:
                    rotEuler[ValueIndex] += increment;
                    break;
                case PoseControlValueTypes.Scale:
                    kbScale += increment;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
            keyboardTransform.localScale = Vector3.one * kbScale;
            keyboardTransform.position = kbPosition;
            keyboardTransform.eulerAngles = rotEuler;

            UpdateText(keyboardTransform);
        }

        private static void GetTransformDetails(Transform transform, out Vector3 position, out Vector3 eulerRotation, out float scale)
        {
            scale = transform.localScale.x;
            position = transform.position;
            eulerRotation = transform.eulerAngles;
        }
    }

    [SerializeField]
    private List<PoseControls> poseControlsList;
    private OVRVirtualKeyboard _virtualKeyboard;
    private bool _foundVirtualKeyboard;

    public void Awake()
    {
        foreach (var control in poseControlsList)
        {
            control.PlusButton.onClick.AddListener(() =>
            {
                UpdateValue(control);
            });
            control.MinusButton.onClick.AddListener(() =>
            {
                UpdateValue(control, true);
            });
        }
    }

    private void Update()
    {
        if (!TryGetVirtualKeyboard(out OVRVirtualKeyboard keyboard))
        {
            return;
        }
        foreach (var control in poseControlsList)
        {
            control.UpdateText(keyboard.transform);
        }
    }

    private void UpdateValue(PoseControls control, bool decreaseValue = false)
    {
        if (!TryGetVirtualKeyboard(out OVRVirtualKeyboard keyboard))
        {
            return;
        }
        control.ModifyTransform(keyboard.transform, decreaseValue);
    }

    private bool TryGetVirtualKeyboard(out OVRVirtualKeyboard ovrVirtualKeyboard)
    {
        if (_foundVirtualKeyboard && _virtualKeyboard == null)
        {
            _foundVirtualKeyboard = false;
        }
        if (!_foundVirtualKeyboard)
        {
            _virtualKeyboard = FindAnyObjectByType<OVRVirtualKeyboard>();
            _foundVirtualKeyboard = _virtualKeyboard != null;
        }

        ovrVirtualKeyboard = _virtualKeyboard;
        return _foundVirtualKeyboard;
    }

}
