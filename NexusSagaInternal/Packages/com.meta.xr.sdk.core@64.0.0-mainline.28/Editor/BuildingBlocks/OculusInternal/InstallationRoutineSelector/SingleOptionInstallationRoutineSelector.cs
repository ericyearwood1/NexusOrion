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

#if OVR_INTERNAL_CODE // BB_INTERFACE

namespace Meta.XR.BuildingBlocks.Editor
{
    public class SingleOptionInstallationRoutineSelector : InstallationRoutineSelector
    {
        public override string Name => "Single Option";

        public override InstallationRoutine Get(string groupName,
            IEnumerable<InstallationRoutine> availableInstallationRoutines) =>
            availableInstallationRoutines.ToArray() switch
            {
                { Length: 1 } routines => routines[0],
                _ => throw new InvalidOperationException($"{Name} InstallationRoutineSelector only supports having one available InstallationRoutine")
            };
    }
}

#endif
