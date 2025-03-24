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
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;
using UnityEngine.UI;

[TestFixture]
[Category("OnCall:xrinput_text_entry")]
public class OVRVirtualKeyboardSamplePoseControllerInternalTest
{

    [Test]
    public void ModifyTransformUpdatesTransformAndText([Values] OVRVirtualKeyboardSamplePoseControllerInternal.PoseControlValueTypes valueType)
    {

        var go = new GameObject();
        var textField = go.AddComponent<Text>();

        var controls = new OVRVirtualKeyboardSamplePoseControllerInternal.PoseControls
        {
            ValueType = valueType,
            StepSize = ((int)valueType + 1) * 1.0f, // +1 to avoid a 0 increment
            Value = textField,
            Formatting = "0.00"
        };

        // Increase twice and decrease once to land on single increment value
        controls.ModifyTransform(go.transform);
        controls.ModifyTransform(go.transform);
        controls.ModifyTransform(go.transform, true);
        // Touch y and z too (scale is ignored)
        controls.ValueIndex += 1;
        controls.ModifyTransform(go.transform);
        controls.ValueIndex += 1;
        controls.ModifyTransform(go.transform);

        var targetTransform = go.transform;
        switch (valueType)
        {
            case OVRVirtualKeyboardSamplePoseControllerInternal.PoseControlValueTypes.Position:
                Assert.That(targetTransform.position, Is.EqualTo(new Vector3(1.0f, 1.0f, 1.0f)).Using(Vector3EqualityComparer.Instance));
                Assert.That(textField.text, Is.EqualTo("1.00"));
                break;
            case OVRVirtualKeyboardSamplePoseControllerInternal.PoseControlValueTypes.Rotation:
                Assert.That(targetTransform.eulerAngles, Is.EqualTo(new Vector3(2.0f, 2.0f, 2.0f)).Using(Vector3EqualityComparer.Instance));
                Assert.That(textField.text, Is.EqualTo("2.00"));
                break;
            case OVRVirtualKeyboardSamplePoseControllerInternal.PoseControlValueTypes.Scale:
                Assert.That(targetTransform.localScale, Is.EqualTo(Vector3.one * 10.0f).Using(Vector3EqualityComparer.Instance));
                Assert.That(textField.text, Is.EqualTo("10.00"));
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(valueType), valueType, null);
        }
    }
}
