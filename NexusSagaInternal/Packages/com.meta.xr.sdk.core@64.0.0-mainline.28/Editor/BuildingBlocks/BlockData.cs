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
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.SceneManagement;

#if OVR_INTERNAL_CODE
using JetBrains.Annotations;
#endif // OVR_INTERNAL_CODE

namespace Meta.XR.BuildingBlocks.Editor
{
    public class BlockData : BlockBaseData
    {
        [SerializeField] internal GameObject prefab;
        public GameObject Prefab => prefab;
        protected virtual bool UsesPrefab => true;
        internal bool GetUsesPrefab => UsesPrefab;

        [SerializeField] internal List<string> externalBlockDependencies;
        [SerializeField] internal List<string> dependencies;

#if OVR_INTERNAL_CODE
        [ItemCanBeNull]
#endif // OVR_INTERNAL_CODE
        public IEnumerable<BlockData> Dependencies =>
            (dependencies ?? Enumerable.Empty<string>())
            .Concat(externalBlockDependencies ?? Enumerable.Empty<string>())
            .Select(Utils.GetBlockData);

        [SerializeField] internal List<string> packageDependencies;
        public virtual IEnumerable<string> PackageDependencies => packageDependencies ?? Enumerable.Empty<string>();

        [Tooltip("Indicates whether only one instance of this block can be installed per scene.")]
        [SerializeField]
        internal bool isSingleton;

        [Tooltip("(Optional) Briefly write how this block should be used.")]
        [TextArea(5, 40)]
        [SerializeField]
        internal string usageInstructions;

        public string UsageInstructions => usageInstructions;
        public bool IsSingleton => isSingleton;

#if OVRPLUGIN_TESTING
        #region Validation
        [ContextMenu("Validate")]
        internal override void Validate()
        {
            base.Validate();

            if (UsesPrefab)
            {
                Assert.IsNotNull(Prefab, $"{nameof(Prefab)} cannot be null");
                Assert.IsNull(Prefab.GetComponent<BuildingBlock>(),
                    $"{nameof(Prefab)} must not contain the {nameof(BuildingBlock)} component, it'll be added dynamically");
            }
            else
            {
                Assert.IsNull(Prefab, $"{nameof(Prefab)} must be null as it is marked as Not Using Prefab");
            }

            Assert.IsFalse(HasCyclicDependencies(), $"{nameof(Dependencies)} cannot have cycles");
            Assert.IsFalse(dependencies.HasDuplicates(), $"{nameof(Dependencies)} cannot have duplicates");
            Assert.IsFalse(dependencies.Any(string.IsNullOrEmpty), $"{nameof(dependencies)} cannot have null or empty");

            try
            {
                Assert.IsFalse(PackageDependencies.HasDuplicates(),
                    $"{nameof(PackageDependencies)} cannot have duplicates");
                Assert.IsTrue(PackageDependencies.All(Utils.IsValidPackageId),
                    $"{nameof(PackageDependencies)} must follow the reverse domain name notation");
                Assert.IsFalse(PackageDependencies.Any(string.IsNullOrEmpty),
                    $"{nameof(PackageDependencies)} cannot have null or empty");
            }
            catch (Exception)
            {
                // PackageDependencies may not be available in InterfaceBlockData as it's contained in InstallationRoutines
                // which may be stored in different projects and it shouldn't break the validation
            }

            Assert.IsFalse(externalBlockDependencies.Any(string.IsNullOrEmpty), $"{nameof(externalBlockDependencies)} cannot have null or empty");
        }

        internal bool HasCyclicDependencies()
        {
            var visited = new Dictionary<BlockData, bool>();
            var recStack = new Dictionary<BlockData, bool>();

            visited[this] = true;
            recStack[this] = true;

            foreach (var dependency in Dependencies)
            {
                if (dependency == null)
                {
                    continue;
                }

                if (IsCyclicUntil(dependency, visited, recStack))
                {
                    return true;
                }
            }

            return false;
        }

        private static bool IsCyclicUntil(BlockData blockData,
            IDictionary<BlockData, bool> visited,
            IDictionary<BlockData, bool> recStack)
        {
            if (recStack.TryGetValue(blockData, out var hasRec) && hasRec)
            {
                return true;
            }

            if (visited.TryGetValue(blockData, out var hasVisited) && hasVisited)
            {
                return false;
            }

            visited[blockData] = true;
            recStack[blockData] = true;

            if (blockData.Dependencies
                .Where(dependency => dependency != null)
                .Any(dependency => IsCyclicUntil(dependency, visited, recStack))
                )
            {
                return true;
            }

            recStack[blockData] = false;
            return false;
        }
        #endregion
#endif // OVRPLUGIN_TESTING

        internal override void AddToProject(GameObject selectedGameObject = null, Action onInstall = null)
        {
            using (new OVREditorUtils.UndoScope($"Install {Utils.BlockPublicTag} {BlockName}"))
            {
                InstallPackagesAndBlockData(selectedGameObject, onInstall);
            }
        }

        [ContextMenu("Install")]
        private void ContextMenuInstall()
        {
            AddToProject(null, null);
        }

        private void InstallPackagesAndBlockData(GameObject selectedGameObject = null, Action onInstall = null)
        {
            try
            {
                var (success, nInstalled) = InstallPackageDependencies();

                if (success && nInstalled > 0)
                {
                    BlockInstaller.RequestInstallation(this, selectedGameObject);
                }
                else
                {
                    InstallWithDependenciesAndCommit(selectedGameObject);
                    onInstall?.Invoke();
                }
            }
            catch (Exception e)
            {
                OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.InstallBlockData)
                    .SetResult(OVRPlugin.Qpl.ResultType.Fail)
                    .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.BlockId, Id)
                    .AddAnnotationIfNotNullOrEmpty(OVRTelemetryConstants.BB.AnnotationType.Error, e.Message)
                    .Send();
                throw;
            }
        }

        internal void InstallWithDependenciesAndCommit(GameObject selectedGameObject = null)
        {
            Exception installException = null;
            try
            {
#if OVRPLUGIN_TESTING
                Validate();
#endif // OVRPLUGIN_TESTING
                var installedObjects = InstallWithDependencies(selectedGameObject);

                SaveScene();
                FixSetupRules();

                EditorApplication.delayCall += () => { Utils.SelectBlocksInScene(installedObjects); };
            }
            catch (Exception e)
            {
                installException = e;
                throw;
            }
            finally
            {
                OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.InstallBlockData)
                    .SetResult(installException == null ? OVRPlugin.Qpl.ResultType.Success : OVRPlugin.Qpl.ResultType.Fail)
                    .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.BlockId, Id)
                    .AddAnnotationIfNotNullOrEmpty(OVRTelemetryConstants.BB.AnnotationType.Error, installException?.Message)
                    .Send();
            }
        }

        internal static void FixSetupRules()
        {
            var buildTargetGroup = BuildPipeline.GetBuildTargetGroup(EditorUserBuildSettings.activeBuildTarget);
            UpdateTasks(buildTargetGroup, FixTasks);
        }

        private static void FixTasks(OVRConfigurationTaskProcessor processor)
        {
            OVRProjectSetup.FixTasks(processor.BuildTargetGroup, tasks => tasks
                .Where(task =>
                    !task.IsDone(processor.BuildTargetGroup)
                    && !task.IsIgnored(processor.BuildTargetGroup)
                    && task.Level.GetValue(processor.BuildTargetGroup) == OVRProjectSetup.TaskLevel.Required)
                .ToList(), blocking: false, onCompleted: AfterFixApply);
        }

        private static void UpdateTasks(BuildTargetGroup buildTargetGroup,
            Action<OVRConfigurationTaskProcessor> onCompleted = null)
        {
            OVRProjectSetup.UpdateTasks(buildTargetGroup, logMessages: OVRProjectSetup.LogMessages.Disabled,
                blocking: false, onCompleted: onCompleted);
        }

        private static void AfterFixApply(OVRConfigurationTaskProcessor processor)
        {
            AssetDatabase.SaveAssets();
            UpdateTasks(processor.BuildTargetGroup);
        }

        internal override bool CanBeAdded => !HasMissingDependencies && !IsSingletonAndAlreadyPresent;
        internal bool HasMissingDependencies => GetMissingDependencies.Any();

        private IEnumerable<string> GetMissingDependencies =>
            (dependencies ?? Enumerable.Empty<string>())
            .Concat(
                PackageDependencies.All(OVRProjectSetupUtils.IsPackageInstalled)
                    ? externalBlockDependencies ?? Enumerable.Empty<string>()
                    : Enumerable.Empty<string>())
            .Where(dependencyId => Utils.GetBlockData(dependencyId) == null);

        private bool IsSingletonAndAlreadyPresent => IsSingleton && IsBlockPresentInScene();

        private (bool, int) InstallPackageDependencies()
        {
            var nInstalled = PackageDependencies.Count(packageId => InstallPackage(packageId) == InstallPackageStatus.Installed);
            return (true, nInstalled);
        }

        private enum InstallPackageStatus
        {
            AlreadyInstalled,
            Installed
        }

        private InstallPackageStatus InstallPackage(string packageId)
        {
            if (OVRProjectSetupUtils.IsPackageInstalled(packageId))
            {
                return InstallPackageStatus.AlreadyInstalled;
            }

            var installed = OVRProjectSetupUtils.InstallPackage(packageId);

            if (!installed)
            {
                throw new InvalidOperationException(
                    $"Installation of package dependency {packageId} failed for block {BlockName}.");
            }

            return InstallPackageStatus.Installed;
        }

        internal List<GameObject> InstallWithDependencies(GameObject selectedGameObject = null)
        {
            if (IsSingletonAndAlreadyPresent)
            {
                throw new InvalidOperationException(
                    $"Block {BlockName} is a singleton and already present in the scene so it cannot be installed.");
            }

            if (HasMissingDependencies)
            {
                throw new InvalidOperationException($"A dependency of block {BlockName} is not present in the project: {string.Join(", ", GetMissingDependencies)}");
            }

            using (new OVREditorUtils.UndoScope($"Install {Utils.BlockPublicTag} {BlockName}"))
            {
                InstallDependencies(Dependencies, selectedGameObject);
                return Install(selectedGameObject);
            }
        }

        internal virtual List<GameObject> Install(GameObject selectedGameObject = null)
        {
            return InstallBlock<BuildingBlock>(selectedGameObject);
        }

        internal List<GameObject> InstallBlock<T>(GameObject selectedGameObject) where T : BuildingBlock
        {
            var spawnedObjects = InstallRoutine(selectedGameObject);

            foreach (var spawnedObject in spawnedObjects)
            {
                var block = Undo.AddComponent<T>(spawnedObject);
                SetupBlockComponent(block);
                while (UnityEditorInternal.ComponentUtility.MoveComponentUp(block))
                {
                }
                Undo.RegisterCompleteObjectUndo(block, $"Setup {nameof(T)}");

                OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.AddBlock)
                    .AddBlockInfo(block)
                    .AddSceneInfo(spawnedObject.scene)
                    .Send();
            }

            return spawnedObjects;
        }

        protected virtual void SetupBlockComponent<T>(T block) where T : BuildingBlock
        {
            block.blockId = Id;
            block.version = Version;
        }

        protected virtual List<GameObject> InstallRoutine(GameObject selectedGameObject)
        {
            var instance = Instantiate(Prefab, Vector3.zero, Quaternion.identity);
            instance.SetActive(true);
            instance.name = $"{Utils.BlockPublicTag} {BlockName}";
            Undo.RegisterCreatedObjectUndo(instance, "Create " + instance.name);
            return new List<GameObject> { instance };
        }

        private static void InstallDependencies(IEnumerable<BlockData> dependencies, GameObject selectedGameObject = null)
        {
            foreach (var dependency in dependencies)
            {
                if (IsBlockPresentInScene(dependency.Id))
                {
                    continue;
                }

                dependency.InstallWithDependencies(selectedGameObject);
            }
        }

        private static bool IsBlockPresentInScene(string blockId)
        {
            return FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Any(x => x.BlockId == blockId);
        }

        private bool IsBlockPresentInScene()
        {
            return IsBlockPresentInScene(Id);
        }

        internal bool IsUpdateAvailableForBlock(BuildingBlock block) => Version > block.Version;

        internal void UpdateBlockToLatestVersion(BuildingBlock block)
        {
            if (!IsUpdateAvailableForBlock(block))
            {
                throw new InvalidOperationException(
                    $"Block {BlockName} is already in the latest version.");
            }

            if (IsSingleton)
            {
                foreach (var instance in this.GetBlocks())
                {
                    DestroyImmediate(instance.gameObject);
                }
            }
            else
            {
                DestroyImmediate(block.gameObject);
            }

            InstallWithDependenciesAndCommit();

            OVRTelemetry.Start(OVRTelemetryConstants.BB.MarkerId.UpdateBlock)
                .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.BlockId, Id)
                .AddAnnotation(OVRTelemetryConstants.BB.AnnotationType.Version, Version.ToString())
                .Send();
        }

        private static void SaveScene()
        {
            if (!IsCurrentSceneSaved())
            {
                return;
            }

            var activeScene = SceneManager.GetActiveScene();
            EditorSceneManager.MarkSceneDirty(activeScene);
            EditorSceneManager.SaveScene(activeScene);
        }

        private static bool IsCurrentSceneSaved()
        {
            var scenePath = SceneManager.GetActiveScene().path;
            return !string.IsNullOrEmpty(scenePath);
        }


        internal override bool OverridesInstallRoutine
        {
            get
            {
                var derivedMethodInfo = GetType().GetMethod(nameof(InstallRoutine),
                    BindingFlags.Instance | BindingFlags.NonPublic, null, new[] { typeof(GameObject) }, null);
                return derivedMethodInfo != null &&
                       derivedMethodInfo != derivedMethodInfo.GetBaseDefinition();
            }
        }
    }
}
