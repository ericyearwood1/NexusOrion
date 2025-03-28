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

using UnityEditor;
using UnityEngine;
using static Meta.XR.Editor.UserInterface.Utils;

namespace Meta.XR.Editor.UserInterface
{
    internal static class Styles
    {
        public static class Colors
        {
            public static readonly Color ExperimentalColor = HexToColor("#eba333");
            public static readonly Color NewColor = HexToColor("#ffc75d");
            public static readonly Color DarkBlue = HexToColor("#48484d");
            public static readonly Color BrightGray = HexToColor("#c4c4c4");
            public static readonly Color LightGray = HexToColor("#aaaaaa");
            public static readonly Color DarkGray = HexToColor("#3e3e3e");
            public static readonly Color DarkGraySemiTransparent = HexToColor("#3e3e3eaa");
            public static readonly Color DarkGrayHover = HexToColor("#4e4e4e");
            public static readonly Color DarkGrayActive = HexToColor("#5d5d5d");
            public static readonly Color CharcoalGray = HexToColor("#1d1d1d");
            public static readonly Color CharcoalGraySemiTransparent = HexToColor("#1d1d1d80");
            public static readonly Color OffWhite = HexToColor("#dddddd");
            public static readonly Color ErrorColor = HexToColor("ed5757");
            public static readonly Color ErrorColorSemiTransparent = HexToColor("ed575780");
            public static readonly Color WarningColor = HexToColor("e9974e");
            public static readonly Color InfoColor = HexToColor("c4c4c4");
            public static readonly Color SuccessColor = HexToColor("4ee99e");
            public static readonly Color DebugColor = HexToColor("#66aaff");

#if OVR_INTERNAL_CODE
            public static readonly Color InternalColor = HexToColor("#66aaff");
#endif
        }

        public class GUIStylesContainer
        {
            public readonly GUIStyle BoldLabel = new GUIStyle(EditorStyles.boldLabel);

            public readonly GUIStyle BoldLabelHover = new GUIStyle(EditorStyles.boldLabel)
            {
                normal = { textColor = Color.white }
            };

            public readonly GUIStyle MiniButton = new GUIStyle(EditorStyles.miniButton)
            {
                clipping = TextClipping.Overflow,
                fixedHeight = Constants.MiniIconHeight,
                fixedWidth = Constants.MiniIconHeight,
                margin = new RectOffset(Constants.MiniPadding, Constants.MiniPadding, Constants.MiniPadding, Constants.MiniPadding),
                padding = new RectOffset(Constants.MiniPadding, Constants.MiniPadding, Constants.MiniPadding, Constants.MiniPadding)
            };

            public readonly GUIStyle Header = new GUIStyle(EditorStyles.miniLabel)
            {
                fontSize = 12,
                fixedHeight = Constants.LargeMargin + Constants.Margin * 2,
                padding = new RectOffset(Constants.Margin, Constants.Margin, Constants.Margin, Constants.Margin),
                margin = new RectOffset(0, 0, 0, 0),

                wordWrap = true,
                normal =
                {
                    background = Colors.DarkGray.ToTexture()
                }
            };

            public readonly GUIStyle HeaderIconStyle = new GUIStyle()
            {
                fixedHeight = Constants.LargeMargin,
                fixedWidth = Constants.LargeMargin,
                stretchWidth = false,
                alignment = TextAnchor.MiddleCenter
            };

            public readonly GUIStyle HeaderLabel = new GUIStyle(EditorStyles.boldLabel)
            {
                stretchHeight = true,
                fixedHeight = Constants.LargeMargin,
                fontSize = 16,
                normal =
                {
                    textColor = Color.white
                },
                hover =
                {
                    textColor = Color.white
                },
                alignment = TextAnchor.MiddleLeft
            };

            public readonly GUIStyle DialogBox = new GUIStyle()
            {
                padding = new RectOffset(Constants.Margin, Constants.Margin, Constants.Margin, Constants.Margin),
                stretchWidth = true,
                stretchHeight = false,
                normal =
                {
                    background = Colors.CharcoalGraySemiTransparent.ToTexture()
                }
            };

            public readonly GUIStyle DialogTextStyle = new GUIStyle()
            {
                padding = new RectOffset(Constants.MiniPadding, Constants.MiniPadding, Constants.MiniPadding, Constants.MiniPadding),
                stretchWidth = true,
                richText = true,
                fontSize = 11,
                wordWrap = true,
                normal =
                {
                    textColor = Colors.OffWhite
                }
            };

            public readonly GUIStyle DialogIconStyle = new GUIStyle()
            {
                padding = new RectOffset(Constants.MiniPadding, Constants.MiniPadding, Constants.MiniPadding, Constants.MiniPadding),
                fixedHeight = Constants.LargeMargin,
                fixedWidth = Constants.LargeMargin,
                stretchWidth = false,
                alignment = TextAnchor.MiddleCenter
            };

#if OVR_INTERNAL_CODE
            public readonly GUIStyle InternalNoticeTextStyle = new GUIStyle()
            {
                padding = new RectOffset(Constants.Padding + Constants.LargeMargin, Constants.Padding, Constants.Padding, Constants.Padding),
                stretchWidth = true,
                richText = true,
                fontSize = 11,
                wordWrap = true,
                normal =
                {
                    textColor = Colors.DarkGray
                }
            };

            public readonly GUIStyle InternalNoticeBox = new GUIStyle()
            {
                padding = new RectOffset(Constants.Margin, Constants.Margin, Constants.Border, Constants.Border),
                stretchWidth = true,
                stretchHeight = false,
                normal =
                {
                    background = Colors.InternalColor.ToTexture()
                }
            };
#endif

        }

        public static class Contents
        {
            public static readonly TextureContent ConfigIcon =
                TextureContent.CreateContent("ovr_icon_cog.png", TextureContent.Categories.Generic, "Additional options");

            public static readonly TextureContent DocumentationIcon =
                TextureContent.CreateContent("ovr_icon_documentation.png", TextureContent.Categories.Generic,
                    "Go to Documentation");

            public static readonly TextureContent DialogIcon =
                TextureContent.CreateContent("ovr_icon_meta_raw.png", TextureContent.Categories.Generic, null);

#if OVR_INTERNAL_CODE
            public static readonly TextureContent WikiIcon =
                TextureContent.CreateContent("ovr_icon_wiki.png", TextureContent.Categories.GenericInternal, "Internal Wiki");

            public static readonly TextureContent WorkplaceIcon =
                TextureContent.CreateContent("ovr_icon_workplace.png", TextureContent.Categories.GenericInternal, "Workplace");

            public static readonly TextureContent FeedbackIcon =
                TextureContent.CreateContent("ovr_icon_bug.png", TextureContent.Categories.Generic,
                    "Give feedback or report bugs");
#endif
        }

        public static class Constants
        {
            public const float ItemHeight = 48.0f;
            public const float MiniIconHeight = 18.0f;
            public const float SmallIconSize = 16.0f;

            public const int Border = 1;
            public const int Padding = 4;
            public const int MiniPadding = 2;
            public const int Margin = 8;
            public const int MiniMargin = 4;
            public const int LargeMargin = 32;
            public const int TextWidthOffset = 2;
        }

        private static GUIStylesContainer _guiStyles;
        public static GUIStylesContainer GUIStyles => _guiStyles ??= new GUIStylesContainer();
    }
}
