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
using UnityEngine;

namespace Meta.XR.BuildingBlocks
{
    public class PlaceObject : MonoBehaviour
    {
        [SerializeField] private GameObject _prefab;

        private Transform _rightHandAnchorTransform;
        private GameObject _previewPrefab;
        private bool _canBePlaced;
        private RoomMeshEvent _roomMeshEvent;
        private MeshFilter _roomMeshFilter;

        public GameObject Prefab
        {
            get => _prefab;
            set
            {
                _prefab = value;
                PreviewPrefab = _prefab;
            }
        }

        private GameObject PreviewPrefab
        {
            set
            {
                if (PreviewPrefab != null)
                {
                    DestroyImmediate(PreviewPrefab);
                }

                _previewPrefab = Instantiate(value, Vector3.zero, Quaternion.identity);
                _previewPrefab.SetActive(true);
            }
            get => _previewPrefab;
        }

        private void Awake()
        {
            _roomMeshEvent = FindObjectOfType<RoomMeshEvent>();
            _roomMeshEvent.OnRoomMeshLoadCompleted.AddListener(OnRoomMeshReady);
        }

        private void Start()
        {
            var cameraRig = FindObjectOfType<OVRCameraRig>();
            if (cameraRig == null)
            {
                throw new NullReferenceException("Could not found OVRCameraRig.");
            }

            _rightHandAnchorTransform = cameraRig.rightControllerAnchor;
            PreviewPrefab = Prefab;
            Prefab.SetActive(false);
        }

        private void Update()
        {
            if (_roomMeshFilter == null)
            {
                return;
            }

            Ray ray = new Ray(_rightHandAnchorTransform.position, _rightHandAnchorTransform.forward);
            Debug.DrawRay(ray.origin, ray.direction);
            if (Physics.Raycast(ray, out var hit, 100) &&
                hit.collider.gameObject == _roomMeshFilter.gameObject)
            {
                PreviewPrefab.transform.position = hit.point;
                PreviewPrefab.transform.up = hit.normal;
                _canBePlaced = true;
                return;
            }

            _canBePlaced = false;
        }

        private void OnRoomMeshReady(MeshFilter meshFilter) => _roomMeshFilter = meshFilter;

        public void InstantiateObject()
        {
            if (!_canBePlaced) return;
            Instantiate(PreviewPrefab);
        }

        private void OnDestroy() => _roomMeshEvent.OnRoomMeshLoadCompleted.RemoveListener(OnRoomMeshReady);
    }
}

#endif // OVR_INTERNAL_CODE
