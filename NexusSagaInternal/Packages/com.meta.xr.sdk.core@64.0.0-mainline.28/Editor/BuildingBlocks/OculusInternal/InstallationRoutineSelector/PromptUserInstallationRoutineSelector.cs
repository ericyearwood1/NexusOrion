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

namespace Meta.XR.BuildingBlocks.Editor
{
    public class PromptUserInstallationRoutineSelector : InstallationRoutineSelector
    {
        public override string Name => "Prompt User";

        public override InstallationRoutine Get(string groupName, IEnumerable<InstallationRoutine> availableInstallationRoutines)
        {
            var selectedGroupTypes =
                Utils.GetBlocksInScene()
                    .Where(block => (block.GetBlockData() as InterfaceBlockData)?.groupName == groupName
                                    && block.GetInstallationRoutine() != null)
                    .Select(block => block.GetInstallationRoutine())
                    .Select(installationRoutine => installationRoutine.GroupType)
                    .Distinct()
                    .ToArray();

            if (selectedGroupTypes is { Length: > 1 })
            {
                throw new InvalidOperationException(
                    $"Unable to get the installation routine when there are blocks of different type for a single group");
            }

            var installationRoutines =
                availableInstallationRoutines
                    .Where(x => selectedGroupTypes.Length == 0 || x.GroupType == selectedGroupTypes.First())
                    .ToArray();

            return installationRoutines switch
            {
                { Length: 0 } => throw new InvalidOperationException(
                    $"There are no available installation routines"),
                { Length: 1 } => installationRoutines.First(),
                _ => SelectInstallationRoutineFromAlternatives(installationRoutines)
            };
        }

        private static InstallationRoutine SelectInstallationRoutineFromAlternatives(
            IEnumerable<InstallationRoutine> installationRoutines)
        {
            throw new NotImplementedException(
                "SelectFromMultipleAlternatives not yet implemented"); // TODO: Show dialog and let the dev choose
        }
    }
}

#endif
