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
using System.Collections.Generic;
using Meta.XR.Editor.StatusMenu;
using Meta.XR.ImmersiveDebugger.DebugData;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using static Meta.XR.Editor.UserInterface.Styles;
using static Meta.XR.Editor.UserInterface.Styles.Contents;

namespace Meta.XR.ImmersiveDebugger.Editor
{
    internal class Settings : SettingsProvider
    {
        public const string SettingsName = Utils.PublicName;
        public static string SettingsPath => $"{OVRProjectSetupSettingsProvider.SettingsPath}/{SettingsName}";

        public static readonly string Description =
            "Displays the console and track Game Objects, MonoBehaviors and their members in real time within your headset." +
            "\n\nYou can track your components' members by using either of the following methods: " +
            $"\n• <i>In code:</i> Add the <b>{nameof(DebugMember)}</b> attribute to any member you want to track" +
            $"\n• <i>In scene:</i> Add and configure the <b>{nameof(DebugInspector)}</b> component to any GameObject you want to track";

        private static Item.Origins? _lastOrigin = null;
        private bool _activated;

        private Settings(string path, SettingsScope scopes, IEnumerable<string> keywords = null)
            : base(path, scopes, keywords)
        {
        }

        [SettingsProvider]
        public static SettingsProvider CreateProjectValidationSettingsProvider()
        {
            return new Settings(SettingsPath, SettingsScope.Project);
        }

        public override void OnTitleBarGUI()
        {
            Utils.Item.DrawHeaderFromSettingProvider();
        }

#if OVR_INTERNAL_CODE
        private static readonly string InternalNotice =
            $"[Internal] <b>{Utils.PublicName}</b> is currently only available for internal Dogfooding. Planned public release : <b>v65</b>";

        private void ShowInternalNotice()
        {
            EditorGUILayout.BeginHorizontal(GUIStyles.InternalNoticeBox);
            EditorGUILayout.BeginVertical();
            EditorGUILayout.LabelField(InternalNotice, GUIStyles.InternalNoticeTextStyle);
            EditorGUILayout.EndVertical();
            EditorGUILayout.EndHorizontal();
        }
#endif

        public override void OnGUI(string searchContext)
        {
#if OVR_INTERNAL_CODE
            ShowInternalNotice();
#endif

            EditorGUILayout.BeginHorizontal(GUIStyles.DialogBox);
            EditorGUILayout.LabelField(DialogIcon, GUIStyles.DialogIconStyle, GUILayout.Width(GUIStyles.DialogIconStyle.fixedWidth));
            EditorGUILayout.BeginVertical();
            EditorGUILayout.LabelField(Description, GUIStyles.DialogTextStyle);
            EditorGUILayout.EndVertical();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space();

            EditorGUI.indentLevel++;

            var previousLabelWidth = EditorGUIUtility.labelWidth;
            EditorGUIUtility.labelWidth = Styles.Constants.LabelWidth;

            DrawToggle(() => RuntimeSettings.Instance.ImmersiveDebuggerEnabled,
                (val) => RuntimeSettings.Instance.ImmersiveDebuggerEnabled = val,
                new GUIContent("Enable", "Enable Immersive Debugger panels in runtime and allow collecting scripting attributes in Editor"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Behavior", GUIStyles.BoldLabel);
            DrawToggle(() => RuntimeSettings.Instance.ImmersiveDebuggerDisplayAtStartup,
                (val) => RuntimeSettings.Instance.ImmersiveDebuggerDisplayAtStartup = val,
                new GUIContent("Display at Start-up", "Display Immersive Debugger panel when the application starts"));
            DrawPopup(() => RuntimeSettings.Instance.ImmersiveDebuggerToggleDisplayButton,
                (val) => RuntimeSettings.Instance.ImmersiveDebuggerToggleDisplayButton = val,
                new GUIContent("Toggle Display Input Button", "Customize the input button to show/hide the Immersive Debugger panel in runtime"));
            DrawToggle(() => RuntimeSettings.Instance.FollowOverride,
                (val) => RuntimeSettings.Instance.FollowOverride = val,
                new GUIContent("Follow Camera Rig Translation", "Whether or not the Immersive Debugger panel follows the player by default"));
            DrawToggle(() => RuntimeSettings.Instance.RotateOverride,
                (val) => RuntimeSettings.Instance.RotateOverride = val,
                new GUIContent("Follow Camera Rig Rotation", "Whether or not the Immersive Debugger panel rotates with the player by default"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Default Toggles", GUIStyles.BoldLabel);
            DrawToggle(() => RuntimeSettings.Instance.ShowInspectors,
                (val) => RuntimeSettings.Instance.ShowInspectors = val,
                new GUIContent("Show Inspectors", "Display Inspectors panel by default"));
            DrawToggle(() => RuntimeSettings.Instance.ShowConsole,
                (val) => RuntimeSettings.Instance.ShowConsole = val,
                new GUIContent("Show Console", "Display Console by default"));
            DrawToggle(() => RuntimeSettings.Instance.ShowInfoLog,
                (val) => RuntimeSettings.Instance.ShowInfoLog = val,
                new GUIContent("Show Info Logs in Console", "Show Info Logs in Console by default"));
            DrawToggle(() => RuntimeSettings.Instance.ShowWarningLog,
                (val) => RuntimeSettings.Instance.ShowWarningLog = val,
                new GUIContent("Show Warning Logs in Console", "Show Warning Logs in Console by default"));
            DrawToggle(() => RuntimeSettings.Instance.ShowErrorLog,
                (val) => RuntimeSettings.Instance.ShowErrorLog = val,
                new GUIContent("Show Error Logs in Console", "Show Error Logs in Console by default"));

            EditorGUIUtility.labelWidth = previousLabelWidth;
            EditorGUI.indentLevel--;
        }

        public static void OpenSettingsWindow(Item.Origins origin)
        {
            _lastOrigin = origin;
            SettingsService.OpenProjectSettings(SettingsPath);
        }

        public override void OnActivate(string searchContext, VisualElement rootElement)
        {
            if (_activated) return;

            _lastOrigin = _lastOrigin ?? Item.Origins.Settings;
            OVRTelemetry.Start(Telemetry.MarkerId.SettingsAccessed)
                .AddAnnotation(Telemetry.AnnotationType.Origin, _lastOrigin.ToString())
                .Send();

            _activated = true;
        }

        private void DrawToggle(Func<bool> get, Action<bool> set, GUIContent content)
        {
            DrawSetting(get, set, content, (guiContent, func) => EditorGUILayout.Toggle(guiContent, func.Invoke()));
        }

        private void DrawPopup<T>(Func<T> get, Action<T> set, GUIContent content)
            where T : Enum
        {
            DrawSetting(get, set, content, ((guiContent, getFunction) => (T)EditorGUILayout.EnumPopup(guiContent, getFunction.Invoke())));
        }

        private void DrawSetting<T>(Func<T> get, Action<T> set, GUIContent content, Func<GUIContent, Func<T>, T> editorGuiFunction)
        {
            EditorGUI.BeginChangeCheck();
            var value = editorGuiFunction.Invoke(content, get);
            if (EditorGUI.EndChangeCheck())
            {
                set.Invoke(value);
                OVRTelemetry.Start(Telemetry.MarkerId.SettingsChanged)
                    .AddAnnotation(Telemetry.AnnotationType.Type, content.text)
                    .AddAnnotation(Telemetry.AnnotationType.Value, value.ToString())
                    .Send();
            }
        }
    }
}

#endif // OVR_INTERNAL_CODE
