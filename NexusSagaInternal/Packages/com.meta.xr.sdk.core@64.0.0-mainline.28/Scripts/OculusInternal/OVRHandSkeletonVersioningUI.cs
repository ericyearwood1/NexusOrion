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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class OVRHandSkeletonVersioningUI : MonoBehaviour
{
    public Button V1Button;
    public Button V2Button;
#if OVR_INTERNAL_CODE
    private void Start()
    {
        V1Button.interactable = false;
    }

    public void UseV1()
    {
        OVRManager.instance.handSkeletonVersion = OVRHandSkeletonVersion.V1;
        V1Button.interactable = false;
        V2Button.interactable = true;
    }

    public void UseV2()
    {
        OVRManager.instance.handSkeletonVersion = OVRHandSkeletonVersion.V2;
        V1Button.interactable = true;
        V2Button.interactable = false;
    }
#endif
}
