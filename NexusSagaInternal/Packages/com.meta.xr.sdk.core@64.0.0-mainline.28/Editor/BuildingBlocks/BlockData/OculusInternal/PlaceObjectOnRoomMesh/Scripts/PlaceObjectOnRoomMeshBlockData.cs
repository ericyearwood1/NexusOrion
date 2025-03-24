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

using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.Events;
using UnityEngine;
using UnityEngine.Events;

namespace Meta.XR.BuildingBlocks.Editor
{
    public class PlaceObjectOnRoomMeshBlockData : BlockData
    {
        protected override List<GameObject> InstallRoutine(GameObject selectedGameObject)
        {
            var placerBlockPrefab = Instantiate(Prefab, Vector3.zero, Quaternion.identity);
            placerBlockPrefab.name = $"{Utils.BlockPublicTag} {BlockName}";
            Undo.RegisterCreatedObjectUndo(placerBlockPrefab, "Create " + placerBlockPrefab.name);

            var placer = placerBlockPrefab.GetComponent<PlaceObject>();

            var buttonMapper = Utils.GetBlocksWithType<ControllerButtonsMapper>().First();
            var buttonAction = new ControllerButtonsMapper.ButtonClickAction
            {
                Title = "Place Object",
                Button = OVRInput.Button.PrimaryIndexTrigger,
                ButtonMode = ControllerButtonsMapper.ButtonClickAction.ButtonClickMode.OnButtonUp,
                Callback = new UnityEvent()
            };
            UnityEventTools.AddPersistentListener(buttonAction.Callback, placer.InstantiateObject);
            buttonMapper.ButtonClickActions.Add(buttonAction);
            Undo.RegisterCompleteObjectUndo(buttonMapper, $"Controller buttons mapping config.");

            return new List<GameObject> { placerBlockPrefab };
        }
    }
}

#endif // OVR_INTERNAL_CODE
