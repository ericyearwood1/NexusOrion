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
using UnityEngine.UI;

namespace Meta.XR.ImmersiveDebugger.UserInterface.Generic
{
    public class Label : Controller
    {
        private Text _text;

        public string Content
        {
            get => _text.text;
            set => _text.text = value;
        }

        protected override void Setup(Controller owner)
        {
            base.Setup(owner);
            _text = GameObject.AddComponent<Text>();

            _text.horizontalOverflow = HorizontalWrapMode.Overflow;
            _text.verticalOverflow = VerticalWrapMode.Overflow;
            _text.text = "";
        }

        public TextStyle TextStyle
        {
            set
            {
                _text.font = value.font;
                _text.fontSize = value.fontSize;
                _text.alignment = value.textAlignement;
                _text.color = value.color;
            }
        }
    }
}

#endif // OVR_INTERNAL_CODE
