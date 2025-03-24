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
using UnityEditor;
using UnityEngine;

public class OVRPassthroughSetup : MonoBehaviour
{
    [MenuItem("Oculus/Internal/Passthrough/Add Background Passthrough")]
    private static void SetupPassthrough()
    {
        var cameraRig = OVRProjectSetupUtils.FindComponentInScene<OVRCameraRig>();
        if (cameraRig != null)
        {
            if (!OVRPassthroughHelper.EnablePassthrough())
            {
                Debug.LogError("Could not enable passthrough for OVRCameraRig");
                return;
            }

            OVRPassthroughHelper.ClearBackground(cameraRig);

            if (!OVRPassthroughHelper.InitPassthroughLayerUnderlay(cameraRig.gameObject))
            {
                Debug.LogError("Could not add an Underlay Passthrough layer to OVRCameraRig");
                return;
            }

            Debug.Log("Passthrough was successfully initialised");
        }
        else
        {
            Debug.LogError(
                "Scene does not contain an OVRCameraRig. Please perform the regular VR scene setup before enabling Passthrough");
        }
    }
}
#endif
