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
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEngine;
using UnityEngine.Assertions;

/// <summary>
/// Editor class defining an interface of OVRCustomExpressionField which is a data class of OVRLinearExpressionWeightGenerator
/// </summary>
[CustomPropertyDrawer(typeof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionField))]
public class OVRLinearExpressionWeightGeneratorCustomExpressionFieldPropertyDrawer : PropertyDrawer
{
    /// <inheritdoc />
    public override void OnGUI(Rect rect, SerializedProperty property, GUIContent label)
    {
        var expressionProp =
            property.FindPropertyRelative(
                nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionField.Expression));
        var coefficientProp =
            property.FindPropertyRelative(
                nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionField.Contribution));
        Assert.IsNotNull(expressionProp);
        Assert.IsNotNull(coefficientProp);

        using (new EditorGUI.PropertyScope(rect, label, property))
        {
            var r = rect;
            r.width = rect.width / 2 - 16;
            EditorGUI.PropertyField(r, expressionProp, GUIContent.none);

            r.x += r.width + 16;
            EditorGUI.PropertyField(r, coefficientProp, GUIContent.none);
        }
    }
}

/// <summary>
/// Editor class defining an interface of OVRCustomExpressionFormula which is a data class of OVRLinearExpressionWeightGenerator
/// </summary>
[CustomPropertyDrawer(typeof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula))]
public class OVRLinearExpressionWeightGeneratorCustomExpressionFormulaPropertyDrawer : PropertyDrawer
{
    private string _nameTitleStr = "Output (custom) blend shape name";
    private string _fieldsTitleStr = "Input components (Oculus blend shapes and their contributions)";

    private GUIContent _normalNameFieldTitle;
    private GUIContent _emptyNameFieldTitle;
    private GUIContent _duplicatedNameFieldTitle;
    private GUIContent _fieldsTitle;
    private GUIContent _multLabel;
    private GUIContent _clmpLabel;

    private float _minLimit;
    private float _maxLimit;

    private float _labelWidth;
    private float _valueWidth = EditorGUIUtility.fieldWidth;
    private float _horizontalSpacing = 5f;
    private GUIStyle _labelStyle = GUI.skin.label;

    /// <summary>
    /// Editor class defining an interface of OVRCustomExpressionFormula which is a data class of OVRLinearExpressionWeightGenerator
    /// </summary>
    public OVRLinearExpressionWeightGeneratorCustomExpressionFormulaPropertyDrawer()
    {
        _emptyNameFieldTitle = EditorGUIUtility.IconContent("console.erroricon.sml",
            "Formula name cannot be empty");
        _emptyNameFieldTitle.text = _nameTitleStr;

        _duplicatedNameFieldTitle = EditorGUIUtility.IconContent("console.erroricon.sml",
            "Formula name should be unique");
        _duplicatedNameFieldTitle.text = _nameTitleStr;

        _normalNameFieldTitle = new GUIContent(_nameTitleStr);
        _fieldsTitle = new GUIContent(_fieldsTitleStr);
        _multLabel = new GUIContent("MULT");
        _clmpLabel = new GUIContent("CLMP");
        _labelWidth = Math.Max(_labelStyle.CalcSize(_multLabel).x, _labelStyle.CalcSize(_clmpLabel).x);

        _minLimit = typeof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula)
            .GetField(nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula.MinValue))
            .GetCustomAttribute<RangeAttribute>()
            .min;

        _maxLimit = typeof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula)
            .GetField(nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula.MaxValue))
            .GetCustomAttribute<RangeAttribute>()
            .max;
    }

    /// <inheritdoc />
    public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
    {
        var fieldsProp =
            property.FindPropertyRelative(nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula.Components));

        var dataHeight =
            EditorGUIUtility.singleLineHeight * 3 +
            EditorGUIUtility.standardVerticalSpacing * 3 +
            EditorGUI.GetPropertyHeight(fieldsProp);

        var extraHeight = EditorGUIUtility.singleLineHeight;

        return dataHeight + extraHeight;
    }

    /// <inheritdoc />
    public override void OnGUI(Rect rect, SerializedProperty property, GUIContent label)
    {
        var targetObject = property.serializedObject.targetObject;
        var targetAsScript = (OVRLinearExpressionWeightGenerator)targetObject;

        var nameProp =
            property.FindPropertyRelative(nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula.Name));
        var fieldsProp =
            property.FindPropertyRelative(nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula.Components));
        var minValueProp =
            property.FindPropertyRelative(
                nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula.MinValue));
        var maxValueProp =
            property.FindPropertyRelative(
                nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula.MaxValue));
        var multiplierProp =
            property.FindPropertyRelative(
                nameof(OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula.Multiplier));


        Assert.IsNotNull(nameProp);
        Assert.IsNotNull(fieldsProp);
        Assert.IsNotNull(minValueProp);
        Assert.IsNotNull(maxValueProp);
        Assert.IsNotNull(multiplierProp);

        var formulaName = nameProp.stringValue;
        var nameFieldTitle = CreateNameTitle(formulaName, targetAsScript);

        using (new EditorGUI.PropertyScope(rect, label, property))
        {
            var r = rect;

            // Output name line
            r.height = EditorGUIUtility.singleLineHeight;
            EditorGUI.PropertyField(r, nameProp, nameFieldTitle);

            // Components (input blendshapes) line
            r.y += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;
            EditorGUI.PropertyField(r, fieldsProp, _fieldsTitle);

            // Multiplier value line
            r.x = rect.x;
            r.y += EditorGUI.GetPropertyHeight(fieldsProp) + EditorGUIUtility.standardVerticalSpacing;

            r.width = _labelWidth;
            EditorGUI.LabelField(r, _multLabel);

            r.x += r.width + _horizontalSpacing + _valueWidth + _horizontalSpacing;
            r.width = rect.width - _labelWidth - _horizontalSpacing - _valueWidth - _horizontalSpacing;
            EditorGUI.Slider(r, multiplierProp, _minLimit, _maxLimit, GUIContent.none);

            // Clamp values line
            r.x = rect.x;
            r.y += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;

            r.width = _labelWidth;
            EditorGUI.LabelField(r, _clmpLabel);

            r.x += r.width + _horizontalSpacing;
            r.width = _valueWidth;
            ValueField(r, minValueProp);

            r.x += r.width + _horizontalSpacing;
            r.width = rect.width - _labelWidth - _valueWidth * 2 - _horizontalSpacing * 3;
            MinMaxSliderField(r, minValueProp, maxValueProp);

            r.x += r.width + _horizontalSpacing;
            r.width = _valueWidth;
            ValueField(r, maxValueProp);
        }
    }

    private GUIContent CreateNameTitle(string formulaName, OVRLinearExpressionWeightGenerator targetAsScript)
    {
        if (formulaName == "")
        {
            Debug.LogError("A weight name cannot be empty", targetAsScript);
            return _emptyNameFieldTitle;
        }

        var weightCount = targetAsScript.WeightCount;
        int num = 0;
        if (weightCount > 0)
        {
            for (int i = 0; i < weightCount; i++)
            {
                if (formulaName == targetAsScript.GetWeightName(i))
                {
                    num++;
                }
            }

            if (num > 1)
            {
                Debug.LogError("A weight name should be unique, the name '" + formulaName + "' is duplicated",
                    targetAsScript);
                return _duplicatedNameFieldTitle;
            }
        }

        return _normalNameFieldTitle;
    }

    private void MinMaxSliderField(Rect r, SerializedProperty minProp, SerializedProperty maxProp)
    {
        float minVal = minProp.floatValue;
        float maxVal = maxProp.floatValue;
        EditorGUI.BeginChangeCheck();
        EditorGUI.MinMaxSlider(r, ref minVal, ref maxVal, _minLimit, _maxLimit);
        if (EditorGUI.EndChangeCheck())
        {
            minProp.floatValue = minVal;
            maxProp.floatValue = maxVal;
        }
    }

    private void ValueField(Rect r, SerializedProperty prop)
    {
        EditorGUI.BeginChangeCheck();
        var val = EditorGUI.FloatField(r, (float)Math.Round(prop.floatValue, 3));
        if (EditorGUI.EndChangeCheck())
        {
            prop.floatValue = val;
        }
    }
}

#if OVR_INTERNAL_CODE

[CustomEditor(typeof(OVRLinearExpressionWeightGenerator))]
public class OVRLinearExpressionWeightGeneratorEditor : Editor
{
    // The best guess for mapping from ARKit blend shapes to Oculus ones
    // ARKit blend shape names are from https://developer.apple.com/documentation/arkit/arfaceanchor/blendshapelocation
    // Oculus blend shapes are from OVRFaceExpressions.ARKitBlendshapes
    // Mapping is from https://www.internalfb.com/code/fbsource/[D39420603-V7]/arvr/projects/xrtx/poes/HandheldMirror/Assets/Scripts/Blendshapes/AvailableBlendshapes.cs?lines=400-485

    private (string, List<OVRFaceExpressions.FaceExpression>)[] ARKitBlendshapesSorted()
    {
        return new[]
            {
                ("EyeBlinkLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesClosedL }),
                ("EyeLookDownLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesLookDownL }),
                ("EyeLookInLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesLookRightL }),
                ("EyeLookOutLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesLookLeftL }),
                ("EyeLookUpLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesLookUpL }),
                ("EyeSquintLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.LidTightenerL }),
                ("EyeWideLeft",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.UpperLidRaiserL }),
                ("EyeBlinkRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesClosedR }),
                ("EyeLookDownRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesLookDownR }),
                ("EyeLookInRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesLookLeftR }),
                ("EyeLookOutRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesLookRightR }),
                ("EyeLookUpRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.EyesLookUpR }),
                ("EyeSquintRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.LidTightenerR }),
                ("EyeWideRight",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.UpperLidRaiserR }),
                ("JawForward",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.JawThrust }),
                ("JawLeft",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.JawSidewaysLeft }),
                ("JawRight",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.JawSidewaysRight }),
                ("JawOpen",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.JawDrop }),
                ("MouthClose",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.LipsToward }),
                ("MouthFunnel",
                    new List<OVRFaceExpressions.FaceExpression>
                    {
                        OVRFaceExpressions.FaceExpression.LipFunnelerLB,
                        OVRFaceExpressions.FaceExpression.LipFunnelerLT,
                        OVRFaceExpressions.FaceExpression.LipFunnelerRB, OVRFaceExpressions.FaceExpression.LipFunnelerRT
                    }),
                ("MouthPucker",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.LipPuckerL, OVRFaceExpressions.FaceExpression.LipPuckerR }),
                ("MouthLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.MouthLeft }),
                ("MouthRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.MouthRight }),
                ("MouthSmileLeft",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.LipCornerPullerL }),
                ("MouthSmileRight",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.LipCornerPullerR }),
                ("MouthFrownLeft",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.LipCornerDepressorL }),
                ("MouthFrownRight",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.LipCornerDepressorR }),
                ("MouthDimpleLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.DimplerL }),
                ("MouthDimpleRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.DimplerR }),
                ("MouthStretchLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.LipStretcherL }),
                ("MouthStretchRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.LipStretcherR }),
                ("MouthRollLower",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.LipSuckLB, OVRFaceExpressions.FaceExpression.LipSuckRB }),
                ("MouthRollUpper",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.LipSuckLT, OVRFaceExpressions.FaceExpression.LipSuckRT }),
                ("MouthShrugLower",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.ChinRaiserB }),
                ("MouthShrugUpper",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.ChinRaiserT }),
                ("MouthPressLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.LipPressorL }),
                ("MouthPressRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.LipPressorR }),
                ("MouthLowerDownLeft",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.LowerLipDepressorL }),
                ("MouthLowerDownRight",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.LowerLipDepressorR }),
                ("MouthUpperUpLeft",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.UpperLipRaiserL }),
                ("MouthUpperUpRight",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.UpperLipRaiserR }),
                ("BrowDownLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.BrowLowererL }),
                ("BrowDownRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.BrowLowererR }),
                ("BrowInnerUp",
                    new List<OVRFaceExpressions.FaceExpression>
                    {
                        OVRFaceExpressions.FaceExpression.InnerBrowRaiserL,
                        OVRFaceExpressions.FaceExpression.InnerBrowRaiserR
                    }),
                ("BrowOuterUpLeft",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.OuterBrowRaiserL }),
                ("BrowOuterUpRight",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.OuterBrowRaiserR }),
                ("CheekPuff",
                    new List<OVRFaceExpressions.FaceExpression>
                        { OVRFaceExpressions.FaceExpression.CheekPuffL, OVRFaceExpressions.FaceExpression.CheekPuffR }),
                ("CheekSquintLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.CheekRaiserL }),
                ("CheekSquintRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.CheekRaiserR }),
                ("NoseSneerLeft",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.NoseWrinklerL }),
                ("NoseSneerRight",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.NoseWrinklerR }),
                ("TongueOut",
                    new List<OVRFaceExpressions.FaceExpression> { OVRFaceExpressions.FaceExpression.TongueOut })
            }
            ;
    }

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        if (GUILayout.Button("[Meta Internal] Load ARKit defaults"))
        {
            var b = (OVRLinearExpressionWeightGenerator)target;

            var template = ARKitBlendshapesSorted();

            b._customExpressionFormulas =
                template.Select(formula => new OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula
                {
                    Name = formula.Item1,
                    Components = formula.Item2.Select(blendshape =>
                        new OVRLinearExpressionWeightGenerator.OVRCustomExpressionField
                        {
                            Expression = blendshape
                        }).ToArray()
                }).ToArray();

            EditorUtility.SetDirty(target);
        }

        if (GUILayout.Button("[Meta Internal] Load OVRFaceExpression defaults"))
        {
            var b = (OVRLinearExpressionWeightGenerator)target;
            b._customExpressionFormulas =
                new OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula[(int)OVRFaceExpressions.FaceExpression
                    .Max];

            for (int i = 0; i < (int)OVRFaceExpressions.FaceExpression.Max; i++)
            {
                b._customExpressionFormulas[i] = new OVRLinearExpressionWeightGenerator.OVRCustomExpressionFormula
                {
                    Name = Enum.GetName(typeof(OVRFaceExpressions.FaceExpression), i),
                    Components = new[]
                    {
                        new OVRLinearExpressionWeightGenerator.OVRCustomExpressionField
                        {
                            Expression = (OVRFaceExpressions.FaceExpression)i,
                        }
                    }
                };
            }

            EditorUtility.SetDirty(target);
        }

        serializedObject.ApplyModifiedProperties();
    }
}
#endif
