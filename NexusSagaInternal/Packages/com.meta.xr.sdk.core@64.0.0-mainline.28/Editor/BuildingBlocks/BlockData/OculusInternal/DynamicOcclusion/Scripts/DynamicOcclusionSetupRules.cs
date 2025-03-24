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

using System.Linq;
using UnityEditor;
using UnityEngine.Rendering;

#if DEPTH_API_SUPPORTED
using Unity.XR.Oculus;
#endif // DEPTH_API_SUPPORTED

namespace Meta.XR.BuildingBlocks.Editor
{

    [InitializeOnLoad]
    internal static class DynamicOcclusionSetupRules
    {
        private static readonly string MinimumUnityVersion = "2022.3.1f";

#if DEPTH_API_SUPPORTED
        private static OculusSettings OculusSettings
        {
            get
            {
                _ = EditorBuildSettings.TryGetConfigObject<OculusSettings>(
                    "Unity.XR.Oculus.Settings", out var settings);
                return settings;
            }
        }
#endif // DEPTH_API_SUPPORTED

        static DynamicOcclusionSetupRules()
        {
            //Vulkan support
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: OVRProjectSetup.TaskGroup.Rendering,
                isDone: _ => !DynamicOcclusionBlockExists ||
                             (PlayerSettings.GetGraphicsAPIs(BuildTarget.Android).Length > 0 &&
                             PlayerSettings.GetGraphicsAPIs(BuildTarget.Android)[0] == GraphicsDeviceType.Vulkan),
                message: "Dynamic Occlusion requires Vulkan to be set as the Default Graphics API",
                fix: _ => PlayerSettings.SetGraphicsAPIs(BuildTarget.Android, new[] { GraphicsDeviceType.Vulkan }),
                fixMessage: "Set Vulkan as Default Graphics API"
            );

            //Scene requirement support
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: OVRProjectSetup.TaskGroup.Features,
                isDone: _ => !DynamicOcclusionBlockExists ||
                             Utils.FindComponentInScene<OVRManager>() == null ||
                             OVRProjectConfig.CachedProjectConfig.sceneSupport == OVRProjectConfig.FeatureSupport.Required,
                message: "Dynamic Occlusion requires Scene feature to be set to required",
                fix: _ =>
                {
                    var projectConfig = OVRProjectConfig.CachedProjectConfig;
                    projectConfig.sceneSupport = OVRProjectConfig.FeatureSupport.Required;
                    OVRProjectConfig.CommitProjectConfig(projectConfig);
                },
                fixMessage: "Enable Scene Required in the project config"
            );

            //Experimental requirement support
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: OVRProjectSetup.TaskGroup.Compatibility,
                isDone: _ => !DynamicOcclusionBlockExists ||
                             Utils.FindComponentInScene<OVRManager>() == null ||
                             OVRProjectConfig.GetProjectConfig().experimentalFeaturesEnabled,
                message: "Dynamic Occlusion requires experimental features to be enabled",
                fix: _ =>
                {
                    var projectConfig = OVRProjectConfig.CachedProjectConfig;
                    projectConfig.experimentalFeaturesEnabled = true;
                    OVRProjectConfig.CommitProjectConfig(projectConfig);
                },
                fixMessage: "Enable experimental features"
            );

            //Unity 2022.3.1 requirement requirement support
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Optional,
                group: OVRProjectSetup.TaskGroup.Compatibility,
                isDone: _ => !DynamicOcclusionBlockExists,
                message: "Dynamic Occlusion requires at least Unity " + MinimumUnityVersion
            );

            // Quest 3 requirement support
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: OVRProjectSetup.TaskGroup.Features,
                isDone: _ => !DynamicOcclusionBlockExists ||
                             (OVRProjectConfig.CachedProjectConfig.targetDeviceTypes.Count == 1 &&
                              OVRProjectConfig.CachedProjectConfig.targetDeviceTypes.Contains(OVRProjectConfig.DeviceType.Quest3)),
                message: "Dynamic Occlusion is only available to Quest 3 devices",
                fix: _ =>
                {
                    var projectConfig = OVRProjectConfig.CachedProjectConfig;
                    projectConfig.targetDeviceTypes.Clear();
                    projectConfig.targetDeviceTypes.Add(OVRProjectConfig.DeviceType.Quest3);
                },
                fixMessage: "Set Quest 3 as a target device"
            );

#if DEPTH_API_SUPPORTED
            //Multiview option
            OVRProjectSetup.AddTask(
                level: OVRProjectSetup.TaskLevel.Required,
                group: OVRProjectSetup.TaskGroup.Compatibility,
                isDone: _ =>
                {
                    if (!DynamicOcclusionBlockExists || OculusSettings == null) return true;
                    return OculusSettings.m_StereoRenderingModeAndroid == OculusSettings.StereoRenderingModeAndroid.Multiview;
                },
                message: "Dynamic Occlusion requires Stereo Rendering Mode to be set to Multiview",
                fix: _ => OculusSettings.m_StereoRenderingModeAndroid =
                    OculusSettings.StereoRenderingModeAndroid.Multiview,
                fixMessage: "Set Stereo Rendering Mode to Multiview"
            );
#endif // DEPTH_API_SUPPORTED
        }

        private static bool DynamicOcclusionBlockExists => Utils.GetBlocksInScene().Any(b => b.BlockId == BlockDataIds.DynamicOcclusion);
    }
}

#endif // OVR_INTERNAL_CODE
