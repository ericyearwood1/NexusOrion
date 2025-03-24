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

using Meta.XR.ImmersiveDebugger.UserInterface.Generic;
using System;
using UnityEngine;

namespace Meta.XR.ImmersiveDebugger.UserInterface
{
    public class ConsoleLine : InteractableController
    {
        private Label _label;
        private Flex _flex;
        private Background _background;
        private Background _pill;
        private LogEntry _entry;

        public LogEntry Entry
        {
            get => _entry;
            set
            {
                _entry = value;
                Label = value.Label;
                PillStyle = value.Severity.PillStyle;
            }
        }

        public string Label
        {
            get => _label.Content;
            set => _label.Content = value;
        }

        public ImageStyle BackgroundStyle
        {
            set
            {
                _background.Sprite = value.sprite;
                _background.Color = value.color;
                _background.PixelDensityMultiplier = value.pixelDensityMultiplier;
            }
        }

        public ImageStyle PillStyle
        {
            set
            {
                _pill.Sprite = value.sprite;
                _pill.Color = value.color;
                _pill.PixelDensityMultiplier = value.pixelDensityMultiplier;
            }
        }

        protected override void Setup(Controller owner)
        {
            base.Setup(owner);

            // Background
            _background = Append<Background>("background");
            _background.LayoutStyle = Style.Load<LayoutStyle>("Fill");
            BackgroundStyle = Style.Load<ImageStyle>("ConsoleLineBackground");

            // Flex
            _flex = Append<Flex>("line");
            _flex.LayoutStyle = Style.Load<LayoutStyle>("ConsoleLineFlex");

            // Pill
            _pill = _flex.Append<Background>("pill");
            _pill.LayoutStyle = Style.Load<LayoutStyle>("PillVertical");

            // Label
            _label = _flex.Append<Label>("log");
            _label.LayoutStyle = Style.Load<LayoutStyle>("ConsoleLineLabel");
            _label.TextStyle = Style.Load<TextStyle>("ConsoleLineLabel");
        }
    }
}

#endif // OVR_INTERNAL_CODE
