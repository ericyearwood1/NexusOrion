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
using Meta.XR.GuidedSetups.Editor;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    public class DynamicOcclusionBlockData : BlockData
    {
#if OVRPLUGIN_TESTING
        internal override bool CanBeAdded => base.CanBeAdded && ValidEditorVersion();
#endif // OVRPLUGIN_TESTING

        protected override List<GameObject> InstallRoutine(GameObject selectedGameObject)
        {
            var gameObjects = base.InstallRoutine(selectedGameObject);

            if (selectedGameObject == null)
            {
                // Install Dummy Cube
                var cubeBlockData = Utils.GetBlockData(BlockDataIds.Cube);
                var cubeBlockObjects = cubeBlockData.Install();

                // Install on Dummy Cube
                selectedGameObject = cubeBlockObjects.First();
            }

            Undo.RegisterFullObjectHierarchyUndo(selectedGameObject, "Apply occlusion.");

            if (!selectedGameObject.TryGetComponent<Renderer>(out var renderer))
            {
                throw new Exception("A Renderer component is missing. Unable to use this surface for occlusion.");
            }

            renderer.sharedMaterial = Prefab.GetComponentInChildren<MeshRenderer>(true).sharedMaterial;

#if DEPTH_API_SUPPORTED && DEPTH_API_UNITY_SUPPORTED
            if (EditorPrefs.GetInt(GuidedSetupDynamicOcclusion.DynamicOcclusionWindowShowKey, 0) == 0)
            {
                GuidedSetupDynamicOcclusion.ShowWindow();
            }
            return gameObjects;
#else
            Undo.PerformUndo();
            throw new Exception($"{BlockName} block doesn't support Unity {Application.unityVersion}. Requires at least 2022.3.1 or 2023.2.");
#endif // DEPTH_API_SUPPORTED && DEPTH_API_UNITY_SUPPORTED
        }

        internal override List<GameObject> Install(GameObject selectedGameObject = null)
        {
            return InstallBlock<DynamicOcclusionBuildingBlock>(selectedGameObject);
        }

#if OVRPLUGIN_TESTING
        private bool ValidEditorVersion()
        {
            var result = Utils.CompareCurrentUnityEditorVersions("2022.3.1");
            if (result == -1) return false;

            int major = int.Parse(Application.unityVersion.Split(".")[0]);
            if (major == 2023)
            {
                result = Utils.CompareCurrentUnityEditorVersions("2023.2");
                if (result == -1) return false;
            }

            return true;
        }
#endif // OVRPLUGIN_TESTING
    }
}

#endif // OVR_INTERNAL_CODE
