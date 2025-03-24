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
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif

namespace Meta.XR.BuildingBlocks
{
    public class PassthroughSettings : MonoBehaviour
    {
        private readonly List<OVRPassthroughLayer> _ptLayerList = new List<OVRPassthroughLayer>();

        [SerializeField] internal OVRPassthroughLayer.ColorMapEditorType colorMapEditorType;

        [Range(-1f, 1f), SerializeField]
        internal float colorMapEditorContrast;

        [Range(-1f, 1f), SerializeField]
        internal float colorMapEditorBrightness;

        [Range(0f, 1f), SerializeField]
        internal float colorMapEditorPosterize;

        [SerializeField]
        internal Gradient colorMapEditorGradient = OVRPassthroughLayer.CreateNeutralColorMapGradient();

        [Range(-1f, 1f), SerializeField]
        internal float colorMapEditorSaturation;

        [SerializeField] internal Texture2D colorLutSourceTexture;

        [SerializeField] internal Texture2D colorLutTargetTexture;

        [SerializeField] [Range(0f, 1f)]
        internal float lutWeight = 1;

        [SerializeField] internal bool flipLutY = true;

        public void TogglePassthrough()
        {
            foreach (var ptLayer in GetPassthroughBlocks())
            {
                ptLayer.enabled = !ptLayer.enabled;
                MarkDirty(ptLayer);
            }
        }

        public void SetPassthroughState(bool state)
        {
            foreach (var ptLayer in GetPassthroughBlocks())
            {
                ptLayer.enabled = state;
                MarkDirty(ptLayer);
            }
        }

        internal void ApplyColorControl()
        {
            foreach (var ptLayer in GetPassthroughBlocks())
            {
                ApplyColorControl(ptLayer);
            }
        }

        private void ApplyColorControl(OVRPassthroughLayer ptLayer)
        {
            ptLayer.colorMapEditorType = colorMapEditorType;

            switch (colorMapEditorType)
            {
                case OVRPassthroughLayer.ColorMapEditorType.None:
                    break;
                case OVRPassthroughLayer.ColorMapEditorType.GrayscaleToColor:
                    ptLayer.colorMapEditorContrast = colorMapEditorContrast;
                    ptLayer.colorMapEditorBrightness = colorMapEditorBrightness;
                    ptLayer.colorMapEditorPosterize = colorMapEditorPosterize;
                    ptLayer.colorMapEditorGradient = colorMapEditorGradient;
                    break;
                case OVRPassthroughLayer.ColorMapEditorType.Custom:
                    break;
                case OVRPassthroughLayer.ColorMapEditorType.Grayscale:
                    ptLayer.colorMapEditorContrast = colorMapEditorContrast;
                    ptLayer.colorMapEditorBrightness = colorMapEditorBrightness;
                    ptLayer.colorMapEditorPosterize = colorMapEditorPosterize;
                    break;
                case OVRPassthroughLayer.ColorMapEditorType.ColorAdjustment:
                    ptLayer.colorMapEditorContrast = colorMapEditorContrast;
                    ptLayer.colorMapEditorBrightness = colorMapEditorBrightness;
                    ptLayer.colorMapEditorSaturation = colorMapEditorSaturation;
                    break;
                case OVRPassthroughLayer.ColorMapEditorType.ColorLut:
                    ptLayer._colorLutSourceTexture = colorLutSourceTexture;
                    ptLayer._flipLutY = flipLutY;
                    ptLayer._lutWeight = lutWeight;
                    break;
                case OVRPassthroughLayer.ColorMapEditorType.InterpolatedColorLut:
                    ptLayer._colorLutSourceTexture = colorLutSourceTexture;
                    ptLayer._colorLutTargetTexture = colorLutTargetTexture;
                    ptLayer._flipLutY = flipLutY;
                    ptLayer._lutWeight = lutWeight;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            MarkDirty(ptLayer);
        }

        private IEnumerable<OVRPassthroughLayer> GetPassthroughBlocks()
        {
            _ptLayerList.Clear();

            foreach (var layer in FindObjectsByType<OVRPassthroughLayer>(FindObjectsSortMode.None))
            {
                if (layer.gameObject.GetComponent<BuildingBlock>() != null)
                {
                    _ptLayerList.Add(layer);
                }
            }

            return _ptLayerList;
        }

        private static void MarkDirty(Component component)
        {
#if UNITY_EDITOR
            EditorUtility.SetDirty(component);
            EditorSceneManager.MarkSceneDirty(component.gameObject.scene);
#endif
        }
    }
}

#endif // OVR_INTERNAL_CODE
