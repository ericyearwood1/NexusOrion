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

namespace Meta.XR.ImmersiveDebugger.UserInterface.Generic
{
#if OVR_INTERNAL_CODE
    // do not remove internal guards as this is not intended for external use
    [CreateAssetMenu(fileName = "ImageStyle", menuName = "Oculus/ImmersiveDebugger/ImageStyle", order = 100)]
#endif
    public class ImageStyle : Style
    {
        public bool enabled = true;
        public Texture2D icon;
        public Sprite sprite;
        public Color color = Color.white;
        public Color colorHover = Color.white;
        public Color colorOff = Color.white;
        public float pixelDensityMultiplier = 1.0f;
    }
}

#endif
