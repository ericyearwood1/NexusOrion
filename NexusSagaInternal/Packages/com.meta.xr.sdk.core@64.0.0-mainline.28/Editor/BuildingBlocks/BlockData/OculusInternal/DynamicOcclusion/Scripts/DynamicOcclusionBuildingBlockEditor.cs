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

using Meta.XR.Editor.Tags;
using UnityEditor;
using UnityEngine;
using Meta.XR.GuidedSetups.Editor;

namespace Meta.XR.BuildingBlocks.Editor
{
    [CustomEditor(typeof(DynamicOcclusionBuildingBlock))]
    public class DynamicOcclusionBuildingBlockEditor : BuildingBlockEditor
    {
        protected override void ShowAdditionals()
        {
            EditorGUILayout.BeginVertical(Styles.GUIStyles.ErrorHelpBox);
#if !DEPTH_API_UNITY_SUPPORTED
            DrawMessageWithIcon(Styles.Contents.ErrorIcon, "Unsupported Unity Editor. Requires 2022.3.1 or 2023.2.");
#endif // DEPTH_API_UNITY_SUPPORTED

#if !DEPTH_API_SUPPORTED
            DrawMessageWithIcon(Styles.Contents.ErrorIcon, "DepthAPI package is missing. Requires com.unity.xr.oculus of version 4.2.0-exp-env-depth.2.");
#endif // DEPTH_API_SUPPORTED

#if DEPTH_API_UNITY_SUPPORTED && DEPTH_API_SUPPORTED
            DrawMessageWithIcon(Styles.Contents.InfoIcon, "Dynamic Occlusion block made some critical changes in project.");
            if (GUILayout.Button("See the changes"))
            {
                GuidedSetupDynamicOcclusion.ShowWindow();
            }
#endif // DEPTH_API_UNITY_SUPPORTED && DEPTH_API_SUPPORTED

            EditorGUILayout.EndVertical();
        }
    }
}

#endif // OVR_INTERNAL_CODE
