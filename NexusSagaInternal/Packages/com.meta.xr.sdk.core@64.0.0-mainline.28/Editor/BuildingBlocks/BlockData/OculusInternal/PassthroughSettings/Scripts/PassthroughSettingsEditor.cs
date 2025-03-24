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
using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    [CustomEditor(typeof(PassthroughSettings))]
    public class PassthroughSettingsEditor : UnityEditor.Editor
    {
        private SerializedProperty _propColorMapEditorContrast;
        private SerializedProperty _propColorMapEditorBrightness;
        private SerializedProperty _propColorMapEditorPosterize;
        private SerializedProperty _propColorMapEditorGradient;
        private SerializedProperty _propColorMapEditorSaturation;
        private SerializedProperty _propColorLutSourceTexture;
        private SerializedProperty _propColorLutTargetTexture;
        private SerializedProperty _propLutWeight;
        private SerializedProperty _propFlipLutY;

        private void OnEnable()
        {
            _propColorMapEditorContrast =
                serializedObject.FindProperty(nameof(PassthroughSettings.colorMapEditorContrast));
            _propColorMapEditorBrightness =
                serializedObject.FindProperty(nameof(PassthroughSettings.colorMapEditorBrightness));
            _propColorMapEditorPosterize =
                serializedObject.FindProperty(nameof(PassthroughSettings.colorMapEditorPosterize));
            _propColorMapEditorSaturation =
                serializedObject.FindProperty(nameof(PassthroughSettings.colorMapEditorSaturation));
            _propColorMapEditorGradient =
                serializedObject.FindProperty(nameof(PassthroughSettings.colorMapEditorGradient));
            _propColorLutSourceTexture =
                serializedObject.FindProperty(nameof(PassthroughSettings.colorLutSourceTexture));
            _propColorLutTargetTexture =
                serializedObject.FindProperty(nameof(PassthroughSettings.colorLutTargetTexture));
            _propLutWeight = serializedObject.FindProperty(nameof(PassthroughSettings.lutWeight));
            _propFlipLutY = serializedObject.FindProperty(nameof(PassthroughSettings.flipLutY));
        }

        public override void OnInspectorGUI()
        {
            var ptSettings = (PassthroughSettings)target;

            if (GUILayout.Button("Toggle Passthrough layers"))
            {
                ptSettings.TogglePassthrough();
            }

            if (GUILayout.Button("Enable all Passthrough layers"))
            {
                ptSettings.SetPassthroughState(true);
            }

            if (GUILayout.Button("Disable all Passthrough layers"))
            {
                ptSettings.SetPassthroughState(false);
            }

            EditorGUILayout.Space();

            EditorGUI.BeginChangeCheck();
            DrawColorControlGUI(ptSettings);
            var changedColorControl = EditorGUI.EndChangeCheck();

            serializedObject.ApplyModifiedProperties();

            if (changedColorControl)
            {
                ptSettings.ApplyColorControl();
            }
        }

        private void DrawColorControlGUI(PassthroughSettings ptSettings)
        {
            // Custom popup for color map type to control order, names, and visibility of types
            var colorMapTypeIndex =
                Array.IndexOf(OVRPassthroughLayerEditor.ColorMapTypes, ptSettings.colorMapEditorType);
            if (colorMapTypeIndex == -1)
            {
                Debug.LogWarning("Invalid color map type encountered");
                colorMapTypeIndex = 0;
            }

            // Dropdown list contains "Custom" only if it is currently selected.
            var colorMapNames = ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.Custom
                ? OVRPassthroughLayerEditor.ColorMapNames
                : OVRPassthroughLayerEditor.SelectableColorMapNames;
            var colorMapLabels = new GUIContent[colorMapNames.Length];
            for (var i = 0; i < colorMapNames.Length; i++)
                colorMapLabels[i] = new GUIContent(colorMapNames[i]);
            var modified = false;
            OVREditorUtil.SetupPopupField(target,
                new GUIContent("Color Control", "The type of color controls applied to this layer"),
                ref colorMapTypeIndex,
                colorMapLabels,
                ref modified);
            ptSettings.colorMapEditorType = OVRPassthroughLayerEditor.ColorMapTypes[colorMapTypeIndex];

            if (ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.Grayscale
                || ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.GrayscaleToColor
                || ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.ColorAdjustment)
            {
                EditorGUILayout.PropertyField(_propColorMapEditorContrast, new GUIContent("Contrast"));
                EditorGUILayout.PropertyField(_propColorMapEditorBrightness, new GUIContent("Brightness"));
            }

            if (ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.Grayscale
                || ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.GrayscaleToColor)
            {
                EditorGUILayout.PropertyField(_propColorMapEditorPosterize, new GUIContent("Posterize"));
            }

            if (ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.ColorAdjustment)
            {
                EditorGUILayout.PropertyField(_propColorMapEditorSaturation, new GUIContent("Saturation"));
            }

            if (ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.GrayscaleToColor)
            {
                EditorGUILayout.PropertyField(_propColorMapEditorGradient, new GUIContent("Colorize"));
            }

            if (ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.ColorLut
                || ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.InterpolatedColorLut)
            {
                var sourceLutLabel = ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.ColorLut
                    ? "LUT"
                    : "Source LUT";

                EditorGUILayout.PropertyField(_propColorLutSourceTexture, new GUIContent(sourceLutLabel));
                OVRPassthroughLayerEditor.PerformLutTextureCheck((Texture2D)_propColorLutSourceTexture
                    .objectReferenceValue);

                if (ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.InterpolatedColorLut)
                {
                    EditorGUILayout.PropertyField(_propColorLutTargetTexture, new GUIContent("Target LUT"));
                    OVRPassthroughLayerEditor.PerformLutTextureCheck((Texture2D)_propColorLutTargetTexture
                        .objectReferenceValue);
                }

                var flipLutYTooltip = "Flip LUT textures along the vertical axis on load. This is needed for LUT " +
                                      "images which have color (0, 0, 0) in the top-left corner. Some color grading systems, " +
                                      "e.g. Unity post-processing, have color (0, 0, 0) in the bottom-left corner, " +
                                      "in which case flipping is not needed.";
                EditorGUILayout.PropertyField(_propFlipLutY, new GUIContent("Flip Vertically", flipLutYTooltip));

                var weightTooltip = ptSettings.colorMapEditorType == OVRPassthroughLayer.ColorMapEditorType.ColorLut
                    ? "Blend between the original colors and the specified LUT. A value of 0 leaves the colors unchanged, a value of 1 fully applies the LUT."
                    : "Blend between the source and the target LUT. A value of 0 fully applies the source LUT and a value of 1 fully applies the target LUT.";
                EditorGUILayout.PropertyField(_propLutWeight, new GUIContent("Blend", weightTooltip));
            }
        }
    }
}

#endif // OVR_INTERNAL_CODE
