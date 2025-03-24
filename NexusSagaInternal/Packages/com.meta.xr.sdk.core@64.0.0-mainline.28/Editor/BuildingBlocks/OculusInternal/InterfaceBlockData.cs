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

#if OVR_INTERNAL_CODE // BB_INTERFACE

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Assertions;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    public sealed class InterfaceBlockData : BlockData
    {
        [SerializeField] internal string groupName;
        public string GroupName => groupName;

        [SerializeField] internal InstallationRoutineSelection installationRoutineSelection;

        private InstallationRoutine SelectedInstallationRoutine =>
            installationRoutineSelection.GetSelected(GroupName, GetAvailableInstallationRoutines());

        protected override bool UsesPrefab => false;

        protected override void SetupBlockComponent<T>(T block)
        {
            base.SetupBlockComponent(block);

            block.installationRoutineId = SelectedInstallationRoutine.Id;
        }

        internal override bool CanBeAdded => HasInstallationRoutine && base.CanBeAdded;

        public override IEnumerable<string> PackageDependencies
        {
            get
            {
                try
                {
                    return base.PackageDependencies.Concat(SelectedInstallationRoutine.PackageDependencies);
                }
                catch (Exception)
                {
                    return Enumerable.Empty<string>();
                }
            }
        }

        protected override List<GameObject> InstallRoutine(GameObject selectedGameObject) =>
            SelectedInstallationRoutine.Install(this, selectedGameObject);

#region InstallationRoutine

        private bool HasInstallationRoutine => GetAvailableInstallationRoutines().Any();

        private IEnumerable<InstallationRoutine> GetAvailableInstallationRoutines() =>
            InstallationRoutine.Registry.Values.Where(x => x.TargetBlockDataId == Id);

#endregion

#if OVRPLUGIN_TESTING

#region Validation

        internal override void Validate()
        {
            base.Validate();

            Assert.IsFalse(string.IsNullOrEmpty(GroupName), $"{nameof(GroupName)} cannot be null or empty");
            Assert.IsTrue(InstallationRoutineSelection.AvailableSelectors
                .Any(availableSelector => availableSelector.Name == installationRoutineSelection.selection),
                $"Unknown selected {nameof(InstallationRoutineSelection)} {installationRoutineSelection.selection}");
        }

#endregion

#endif
    }

    [Serializable]
    public class InstallationRoutineSelection
    {
        internal static readonly List<InstallationRoutineSelector> AvailableSelectors =
            new()
            {
                new SingleOptionInstallationRoutineSelector(),
                new PromptUserInstallationRoutineSelector()
            };

        [SerializeField] internal string selection = AvailableSelectors[0].Name;

        public InstallationRoutine GetSelected(string groupName,
            IEnumerable<InstallationRoutine> availableInstallationRoutines) =>
            AvailableSelectors.First(s => s.Name == selection)
                .Get(groupName, availableInstallationRoutines);
    }
}
#endif // OVR_INTERNAL_CODE // BB_INTERFACE
