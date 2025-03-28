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

using System.Collections.Generic;
using Meta.XR.Editor.StatusMenu;
using Meta.XR.Editor.UserInterface;
using UnityEditor;
using UnityEngine;
using static Meta.XR.Editor.UserInterface.Styles.Colors;

internal class OVRUserSettingsProvider : SettingsProvider
{
    public static string SettingsPath => $"Preferences/{OVREditorUtils.MetaXRPublicName}";

    private OVRUserSettingsProvider(string path, SettingsScope scopes, IEnumerable<string> keywords = null)
        : base(path, scopes, keywords)
    {
    }

    [SettingsProvider]
    public static SettingsProvider CreateProjectValidationSettingsProvider()
    {
        return new OVRUserSettingsProvider(SettingsPath, SettingsScope.User);
    }

    public override void OnGUI(string searchContext)
    {
        EditorGUILayout.Space();

        EditorGUI.indentLevel++;

        var previousLabelWidth = EditorGUIUtility.labelWidth;
        EditorGUIUtility.labelWidth = 256.0f;

        EditorGUILayout.BeginVertical();
        {
            EditorGUILayout.LabelField("Telemetry", EditorStyles.boldLabel);
            using (var check = new EditorGUI.ChangeCheckScope())
            {
                var telemetryEnabled =
                    EditorGUILayout.Toggle(new GUIContent("Enable"), OVRTelemetryConsent.TelemetryEnabled);
                if (check.changed)
                {
                    OVRTelemetryConsent.SetTelemetryEnabled(telemetryEnabled,
                        OVRTelemetryConstants.OVRManager.ConsentOrigins.Settings);
                }
            }

#if OVR_INTERNAL_CODE
#if OVRPLUGIN_TESTING
            using (new Utils.ColorScope(Utils.ColorScope.Scope.All, InternalColor))
            {
                using (var check = new EditorGUI.ChangeCheckScope())
                {
                    var telemetryLogged = EditorGUILayout.Toggle(
                        new GUIContent("Log Markers On Sent"), OVRRuntimeSettings.Instance.TelemetryLogged);

                    if (check.changed)
                    {
                        OVRRuntimeSettings.Instance.TelemetryLogged = telemetryLogged;
                    }
                }
            }
#endif
#endif
        }
        EditorGUILayout.EndVertical();

        EditorGUIUtility.labelWidth = previousLabelWidth;
        EditorGUI.indentLevel--;
    }

    public override void OnTitleBarGUI()
    {
        OVREditorUtils.SettingsItem.DrawHeaderFromSettingProvider();
    }

    public static void OpenSettingsWindow(Item.Origins origin)
    {
        SettingsService.OpenProjectSettings(SettingsPath);
    }
}
