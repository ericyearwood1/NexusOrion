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
using UnityEditor;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{
    public abstract class InstallationRoutine : ScriptableObject, IIdentified
    {
        internal static readonly CachedIdDictionary<InstallationRoutine> Registry = new();

        [SerializeField, OVRReadOnly] internal string id = Guid.NewGuid().ToString();
        public string Id => id;

        [SerializeField] private string targetBlockDataId;
        public string TargetBlockDataId => targetBlockDataId;

        [SerializeField] internal GameObject prefab;
        protected GameObject Prefab => prefab;

        [SerializeField] private string groupType;
        public string GroupType => groupType;

        [SerializeField] private List<string> packageDependencies;
        public IEnumerable<string> PackageDependencies => packageDependencies;

        public virtual List<GameObject> Install(BlockData block, GameObject selectedGameObject)
        {
            var instance = Instantiate(Prefab, Vector3.zero, Quaternion.identity);
            instance.SetActive(true);
            instance.name = $"[BB][{GroupType}] {block.BlockName}";
            Undo.RegisterCreatedObjectUndo(instance, "Create " + instance.name);
            return new List<GameObject> { instance };
        }

#if OVR_INTERNAL_CODE
#region Internal

        [ContextMenu("Assign ID")]
        internal void AssignId()
        {
            id = Guid.NewGuid().ToString();
        }

        [ContextMenu("Copy ID to clipboard")]
        internal void CopyIdToClipboard()
        {
            GUIUtility.systemCopyBuffer = Id;
        }

#endregion
#endif // OVR_INTERNAL_CODE
    }
}

#endif
