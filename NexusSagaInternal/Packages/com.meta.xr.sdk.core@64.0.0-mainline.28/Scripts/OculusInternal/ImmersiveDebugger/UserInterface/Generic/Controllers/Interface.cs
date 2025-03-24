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

using UnityEngine;
using UnityEngine.EventSystems;

namespace Meta.XR.ImmersiveDebugger.UserInterface.Generic
{
    public class Interface : Controller
    {
        private OVRCameraRig _cameraRig;
        private OVRInputModule _inputModule;
        public Cursor Cursor { get; private set; }

        protected virtual bool FollowOverride { get; set; }
        protected virtual bool RotateOverride { get; set; }

        public virtual void Awake()
        {
            Setup(null);

            _cameraRig = FindAnyObjectByType<OVRCameraRig>();

            var cursorObject = new GameObject("cursor");
            cursorObject.transform.SetParent(Transform);
            Cursor = cursorObject.AddComponent<Cursor>();

            _inputModule = FindAnyObjectByType<OVRInputModule>() ?? GameObject.AddComponent<OVRInputModule>();
            _inputModule.m_Cursor ??= Cursor;
            _inputModule.allowActivationOnMobileDevice = true;
            _inputModule.joyPadClickButton = OVRInput.Button.Any;
        }

        private void UpdateTransform()
        {
            var centerEyeAnchor = _cameraRig.centerEyeAnchor;
            if (centerEyeAnchor == null)
            {
                return;
            }

            if (FollowOverride)
            {
                Transform.position = centerEyeAnchor.position;
            }

            if (RotateOverride)
            {
                var euler = centerEyeAnchor.eulerAngles;
                // Only rotating around up axis (means the dashboard cannot roll, and stays parallel to the ground)
                euler.x = 0.0f;
                euler.z = 0.0f;
                Transform.rotation = Quaternion.Euler(euler);
            }
        }

        void UpdateController()
        {
            if (_inputModule == null)
            {
                return;
            }

            _inputModule.rayTransform = OVRInput.GetActiveController() switch
            {
                OVRInput.Controller.LTouch => _cameraRig.leftControllerAnchor,
                OVRInput.Controller.RTouch => _cameraRig.rightControllerAnchor,
                _ => _cameraRig.rightControllerAnchor
            };
        }

        public virtual void LateUpdate()
        {
            if (_cameraRig == null)
            {
                return;
            }

            UpdateTransform();
            UpdateController();
        }
    }
}

#endif
