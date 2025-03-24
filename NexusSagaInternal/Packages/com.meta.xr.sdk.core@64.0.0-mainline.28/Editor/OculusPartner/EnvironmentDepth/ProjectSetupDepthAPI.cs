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

using UnityEditor;
#if USING_XR_SDK_OCULUS
using Unity.XR.Oculus;
#endif
using UnityEngine.SceneManagement;
using UnityEngine;
using UnityEditor.PackageManager;
using UnityEngine.Rendering;

namespace Meta.XR.EnvironmentDepth.Editor
{
    [InitializeOnLoad]
    internal static class ProjectSetupDepthAPI
    {
#if !DEPTH_API_UNITY_SUPPORTED
        private const string _minimumUnityVersion = "2022.3.1f";
#endif
#if !DEPTH_API_SUPPORTED
        private const string _xrOculusRequiredVersion = "com.unity.xr.oculus@4.2.0-exp-env-depth.2";
#endif
        private const OVRProjectSetup.TaskGroup GROUP = OVRProjectSetup.TaskGroup.Rendering;

        private static bool _isPassthroughEnabled;
        private static bool _isScenePermissionSet;
        private static bool _isCurrentSceneUsingDepth;
#if USING_XR_SDK_OCULUS
        private static OculusSettings OculusSettings
        {
            get
            {
                _ = EditorBuildSettings.TryGetConfigObject<OculusSettings>(
                    "Unity.XR.Oculus.Settings", out var settings);
                return settings;
            }
        }
#endif
        static ProjectSetupDepthAPI()
        {
            EditorApplication.hierarchyChanged += OnHierarchyChangedCheckDepthRequirements;
#if DEPTH_API_UNITY_SUPPORTED
            // === Per Project Setup Support
            // Vulkan support
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: GROUP,
                isDone: buildTargetGroup =>
                {
                    if (!_isCurrentSceneUsingDepth) return true;
                    return
                    PlayerSettings.GetGraphicsAPIs(BuildTarget.Android).Length > 0 &&
                        PlayerSettings.GetGraphicsAPIs(BuildTarget.Android)[0] == GraphicsDeviceType.Vulkan;
                },
                message: "DepthAPI requires Vulkan to be set as the Default Graphics API.",
                fix: buildTargetGroup =>
                {
                    PlayerSettings.SetGraphicsAPIs(BuildTarget.Android, new GraphicsDeviceType[] { GraphicsDeviceType.Vulkan });
                },
                fixMessage: "Set Vulkan as Default Graphics API"
            );
#if !DEPTH_API_SUPPORTED
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: OVRProjectSetup.TaskGroup.Compatibility,
                isDone: buildTargetGroup =>
                {
                    if (!_isCurrentSceneUsingDepth) return true;
                    return false;
                },
                fix: buildTargetGroup =>
                {
                    Client.Add(_xrOculusRequiredVersion);
                },
                message: $"DepthAPI requires XR Oculus {_xrOculusRequiredVersion}"
            );
#endif // DEPTH_API_SUPPORTED
#if USING_XR_SDK_OCULUS
            // Multiview option
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: GROUP,
                isDone: buildTargetGroup =>
                {
                    if (!_isCurrentSceneUsingDepth) return true;
                    if (OculusSettings == null) return true;
                    return OculusSettings.m_StereoRenderingModeAndroid == OculusSettings.StereoRenderingModeAndroid.Multiview;
                },
                message: "DepthAPI requires Stereo Rendering Mode to be set to Multiview.",
                fix: buildTargetGroup =>
                {
                    OculusSettings.m_StereoRenderingModeAndroid = OculusSettings.StereoRenderingModeAndroid.Multiview;
                },
                fixMessage: "Set Stereo Rendering Mode to Multiview"
            );
#endif
            // Scene requirement support
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: OVRProjectSetup.TaskGroup.Features,
                isDone: buildTargetGroup =>
                {
                    if (!_isCurrentSceneUsingDepth) return true;
                    return _isScenePermissionSet;
                },
                message: "DepthAPI requires Scene feature to be set to required",
                fix: buildTargetGroup =>
                {
                    var projectConfig = OVRProjectConfig.CachedProjectConfig;
                    projectConfig.sceneSupport = OVRProjectConfig.FeatureSupport.Required;
                    OVRProjectConfig.CommitProjectConfig(projectConfig);
                },
                fixMessage: "Enable Scene Required in the project config"
            );
            // === Per Scene Setup Support
            // Passthrough requirement support
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: OVRProjectSetup.TaskGroup.Compatibility,
                isDone: buildTargetGroup =>
                {
                    if (!_isCurrentSceneUsingDepth) return true;
                    return _isPassthroughEnabled;
                },
                message: "DepthAPI requires the Passthrough feature to be enabled",
                fix: buildTargetGroup =>
                {
                    // this will cascade into other passthrough setup tool tasks
                    var ovrManager = FindComponentInScene<OVRManager>();
                    if (FindComponentInScene<OVRPassthroughLayer>() == null)
                    {
                        ovrManager.gameObject.AddComponent<OVRPassthroughLayer>().overlayType = OVROverlay.OverlayType.Underlay;
                    }

                    EditorUtility.SetDirty(ovrManager.gameObject);
                },
                fixMessage: "Enable Passthrough by adding OVRPassthroughLayer to the scene"
            );
#else // DEPTH_API_UNITY_SUPPORTED
            // Unity minimum version requirement support
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Optional,
                group: OVRProjectSetup.TaskGroup.Compatibility,
                isDone: buildTargetGroup =>
                {
                    if (!_isCurrentSceneUsingDepth) return true;//we only check this if DepthTextureProvider is in the scene
                    return false;
                },
                message: "DepthAPI requires at least Unity " + _minimumUnityVersion
            );
#endif // DEPTH_API_UNITY_SUPPORTED
        }

        private static void OnHierarchyChangedCheckDepthRequirements()
        {
            if (FindComponentInScene<OVRManager>() == null ||
                FindComponentInScene<EnvironmentDepthTextureProvider>() == null)
            {
                _isCurrentSceneUsingDepth = false;
                return;
            }
            _isCurrentSceneUsingDepth = true;
            _isPassthroughEnabled = CheckPassthrough();
            _isScenePermissionSet = CheckScenePermission();
        }

        private static bool CheckPassthrough()
        {
            if (FindComponentInScene<OVRPassthroughLayer>() != null)
            {
                return true;
            }
            return false;
        }

        private static bool CheckScenePermission()
        {
            return OVRProjectConfig.CachedProjectConfig.sceneSupport ==
                                    OVRProjectConfig.FeatureSupport.Required;
        }

        private static T FindComponentInScene<T>() where T : Component
        {
            var scene = SceneManager.GetActiveScene();
            var rootGameObjects = scene.GetRootGameObjects();
            foreach (var rootGameObject in rootGameObjects)
            {
                if (rootGameObject.GetComponent<T>() == null)
                {
                    continue;
                }

                return rootGameObject.GetComponent<T>();
            }
            return null;
        }
    }
}
#endif// OVR_PARTNER_CODE || OVR_INTERNAL_CODE
