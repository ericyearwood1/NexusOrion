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
using Meta.XR.Editor.StatusMenu;
using Meta.XR.Editor.UserInterface;
using Meta.XR.ImmersiveDebugger.DebugData;
using UnityEditor;
using UnityEngine;
using static Meta.XR.Editor.UserInterface.Styles.Colors;
using static Meta.XR.Editor.UserInterface.Styles.Contents;

namespace Meta.XR.ImmersiveDebugger.Editor
{
    [InitializeOnLoad]
    internal static class Utils
    {
        internal const string PublicName = "Immersive Debugger";
        internal const string PublicTag = "[ID]";

        internal static readonly TextureContent.Category ImmersiveDebuggerIcons = new("OculusInternal/ImmersiveDebugger/Icons");
        internal static readonly TextureContent StatusIcon = TextureContent.CreateContent("ovr_icon_idf.png", ImmersiveDebuggerIcons, $"Open {PublicName}");

#if OVR_INTERNAL_CODE
        private const string WikiUrl = "https://www.fburl.com/wiki-id";
        private const string WorkplaceUrl = "https://www.fburl.com/workplace-id";
        private const string FeedbackUrl = "https://www.fburl.com/ppt-feedback";
#endif

        internal static Item Item = new Item()
        {
            Name = PublicName,
            Color = Styles.Colors.AccentColor,
            Icon = StatusIcon,
            InfoTextDelegate = ComputeInfoText,
            PillIcon = null,
            OnClickDelegate = OnStatusMenuClick,
            Order = 2,
            HeaderIcons = new List<Item.HeaderIcon>()
            {
                new Item.HeaderIcon()
                {
                    TextureContent = ConfigIcon,
                    Color = LightGray,
                    Action = null
                },
                new Item.HeaderIcon()
                {
                    TextureContent = DocumentationIcon,
                    Color = LightGray,
                    Action = null
                },
#if OVR_INTERNAL_CODE
                new Item.HeaderIcon()
                {
                    TextureContent = WikiIcon,
                    Color = InternalColor,
                    Action = () => Application.OpenURL(WikiUrl)
                },
                new Item.HeaderIcon()
                {
                    TextureContent = WorkplaceIcon,
                    Color = InternalColor,
                    Action = () => Application.OpenURL(WorkplaceUrl)
                },
                new Item.HeaderIcon()
                {
                    TextureContent = FeedbackIcon,
                    Color = InternalColor,
                    Action = () => Application.OpenURL(FeedbackUrl)
                }
#endif
            }
        };

        static Utils()
        {
            StatusMenu.RegisterItem(Item);
        }

        public static (string, Color?) ComputeInfoText()
        {
            var enabled = RuntimeSettings.Instance.ImmersiveDebuggerEnabled;
            return (enabled ? "Enabled" : "Disabled", null);
        }

        private static void OnStatusMenuClick(Item.Origins origin)
        {
            Settings.OpenSettingsWindow(origin);
        }
    }
}

#endif //OVR_INTERNAL_CODE
