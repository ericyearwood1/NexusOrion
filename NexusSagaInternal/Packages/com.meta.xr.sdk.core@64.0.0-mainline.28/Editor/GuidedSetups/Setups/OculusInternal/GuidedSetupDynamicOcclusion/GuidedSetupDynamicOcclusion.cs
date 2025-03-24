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

namespace Meta.XR.GuidedSetups.Editor
{
    public class GuidedSetupDynamicOcclusion : GuidedSetupBase
    {
        public const string DynamicOcclusionWindowShowKey = "dynamic_occlusion_window_show";
        private bool _dontShowAgain = false;
        private const string ImportantTag = "<color=#ffd74e>[Important]</color>";

        public static void ShowWindow()
        {
            var window = GetWindow<GuidedSetupDynamicOcclusion>();

            var title = "Dynamic Occlusion Building Block";
            var description = ImportantTag +
                              " Dynamic Occlusion Building Block made the following critical changes in the current project.";
            window.SetupWindow(title, description);
        }

        protected override void OnGUI()
        {
            base.OnGUI();

            GuidedSetupBeginVertical();

            var packageInfo =
                "Installed experimental <b>com.unity.xr.oculus</b> of version <b>4.2.0-exp-env-depth.2</b> package.";
            AddBulletedGUIContent(GuidedSetupStyles.ContentStatusType.Warning, new GUIContent(packageInfo));

            var graphicsAPIInfo = "Set the <b>Graphics API</b> to <b>Vulkan</b> only in Unity Player Settings.";
            AddBulletedGUIContent(GuidedSetupStyles.ContentStatusType.Warning, new GUIContent(graphicsAPIInfo));

            var quest3Info = "It's a <b>Quest 3</b> only feature and set the target device to <b>Quest 3</b>.";
            AddBulletedGUIContent(GuidedSetupStyles.ContentStatusType.Warning, new GUIContent(quest3Info));

            EditorGUILayout.Space();
            AddGUIContent(new GUIContent("<b>Additional notes:</b>"));
            AddBulletedGUIContent(new GUIContent(
                "Dynamic Occlusion / DepthAPI requires access to spatial data. <b>Scene Support</b> has been enabled."));
            AddBulletedGUIContent(
                new GUIContent("Minimum required Unity Editor version: <b>2022.3.1</b> or <b>2023.2.</b>"));

            EditorGUILayout.Space();
            EditorGUILayout.BeginHorizontal();
            AddGUIContent(new GUIContent("See"));
            if (EditorGUILayout.LinkButton("DepthAPI documentation"))
            {
                Application.OpenURL("https://developer.oculus.com/documentation/unity/unity-depthapi/");
            }
            AddGUIContent(new GUIContent("for more details."));
            GUILayout.FlexibleSpace();
            EditorGUILayout.EndHorizontal();

            GUILayout.FlexibleSpace();

            if (GUILayout.Button("Ok"))
            {
                Close();
            }

            EditorGUILayout.Space();
            EditorGUILayout.BeginHorizontal();
            _dontShowAgain = EditorGUILayout.Toggle(_dontShowAgain, GUILayout.Width(16));
            AddGUIContent(new GUIContent("Don't show this again on install."));
            EditorGUILayout.EndHorizontal();

            GuidedSetupEndVertical();
        }

        private void OnBecameVisible()
        {
            _dontShowAgain = EditorPrefs.GetInt(DynamicOcclusionWindowShowKey, 0) != 0;
        }

        private void OnDestroy()
        {
            EditorPrefs.SetInt(DynamicOcclusionWindowShowKey, _dontShowAgain ? 1 : 0);
        }
    }
}

#endif // OVR_INTERNAL_CODE
