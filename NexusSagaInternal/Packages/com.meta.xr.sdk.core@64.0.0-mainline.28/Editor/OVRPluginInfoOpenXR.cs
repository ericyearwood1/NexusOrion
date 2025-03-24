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


using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace Oculus.VR.Editor
{
    public enum PluginPlatform
    {
        Android,
        AndroidUniversal,
        AndroidOpenXR,
        OSXUniversal,
        Win,
        Win64,
        Win64OpenXR,
    }

    public struct PluginPlatformInfo
    {
        public string pluginName;
        public string folderName;

        public PluginPlatformInfo(string pluginName, string folderName)
        {
            this.pluginName = pluginName;
            this.folderName = folderName;
        }
    }

    [InitializeOnLoad]
    public class OVRPluginInfoOpenXR : IOVRPluginInfoSupplier
    {
        private static bool _unityRunningInBatchMode;
        private static readonly string _isRestartPendingKey = "OVRPluginInfoOpenXR_IsRestartPending";
        private static readonly string _hasRunOnceKey = "OVRPluginInfoOpenXR_HasRunOnce";

        public static readonly IReadOnlyDictionary<PluginPlatform, PluginPlatformInfo> _pluginInfos =
            new Dictionary<PluginPlatform, PluginPlatformInfo>(){
                {PluginPlatform.AndroidOpenXR, new PluginPlatformInfo("OVRPlugin.aar", "AndroidOpenXR") },
                {PluginPlatform.Win64OpenXR, new PluginPlatformInfo("OVRPlugin.dll", "Win64OpenXR") },
            };

        public bool IsOVRPluginOpenXRActivated() => true;

        public bool IsOVRPluginUnityProvidedActivated() => false;

        static OVRPluginInfoOpenXR()
        {
            EditorApplication.delayCall += DelayCall;
        }

        private static void DelayCall()
        {
            if (Environment.CommandLine.Contains("-batchmode"))
            {
                _unityRunningInBatchMode = true;
            }

            if (GetIsRestartPending())
            {
                return;
            }

            CheckHasPluginChanged(false);
            SetHasRunOnce(true);
        }

        private static void CheckHasPluginChanged(bool forceUpdate)
        {
            string win64Path = GetOVRPluginPath(PluginPlatform.Win64OpenXR);
            string androidPath = GetOVRPluginPath(PluginPlatform.AndroidOpenXR);
            string win64FullPath = Path.GetFullPath(win64Path);
            string androidFullPath = Path.GetFullPath(androidPath);

            string md5Win64Actual = "";
            string md5AndroidActual = "";
            if (File.Exists(win64FullPath))
            {
                md5Win64Actual = GetFileChecksum(win64FullPath);
            }
            if (File.Exists(androidFullPath))
            {
                md5AndroidActual = GetFileChecksum(androidFullPath);
            }

            var projectConfig = OVRProjectConfig.GetProjectConfig();
            if (!forceUpdate && projectConfig.ovrPluginMd5Win64 == md5Win64Actual &&
                projectConfig.ovrPluginMd5Android == md5AndroidActual)
            {
                return;
            }

            if (OVRPluginInfo.IsCoreSDKModifiable())
            {
                if (File.Exists(win64FullPath))
                {
                    PluginImporter win64Plugin = AssetImporter.GetAtPath(win64Path) as PluginImporter;
                    ConfigurePlugin(win64Plugin, PluginPlatform.Win64OpenXR);
                }
                if (File.Exists(androidFullPath))
                {
                    PluginImporter androidPlugin = AssetImporter.GetAtPath(androidPath) as PluginImporter;
                    ConfigurePlugin(androidPlugin, PluginPlatform.AndroidOpenXR);
                }
            }

            projectConfig.ovrPluginMd5Win64 = md5Win64Actual;
            projectConfig.ovrPluginMd5Android = md5AndroidActual;
            OVRProjectConfig.CommitProjectConfig(projectConfig);

            if (!GetHasRunOnce())
            {
                // No restart needed on startup run since the right plugin should already be loaded
                return;
            }

            bool userAgreedToRestart = !_unityRunningInBatchMode && EditorUtility.DisplayDialog(
                "Restart Unity",
                "Changes to OVRPlugin detected. Plugin updates require a restart. Please restart Unity to complete the update.",
                "Restart Editor",
                "Not Now");
            SetIsRestartPending(true);
            if (userAgreedToRestart)
            {
                RestartUnityEditor();
            }
            else
            {
                Debug.LogWarning("OVRPlugin not updated. Restart the editor to update.");
            }
        }

#if OVR_INTERNAL_CODE
        [MenuItem("Oculus/Internal/Force Reload OVRPlugin")]
#endif // OVR_INTERNAL_CODE
        public static void BatchmodeCheckHasPluginChanged()
        {
            CheckHasPluginChanged(true);
        }

        private static string GetOVRPluginPath(PluginPlatform platform)
        {
            if (!_pluginInfos.ContainsKey(platform))
            {
                throw new ArgumentException("Unsupported BuildTarget: " + platform);
            }

            string folderName = _pluginInfos[platform].folderName;
            string pluginName = _pluginInfos[platform].pluginName;
            string rootPluginPath = OVRPluginInfo.GetPluginRootPath();
            string pluginPath = Path.Combine(rootPluginPath, folderName, pluginName);

#if OVR_INTERNAL_CODE
            // INTERNAL-ONLY: If plugin doesn't exist at the target path and sdk is modifiable, check the legacy path
            string fullPackagePath = Path.GetFullPath(pluginPath);
            if (!File.Exists(fullPackagePath) && OVRPluginInfo.IsCoreSDKModifiable())
            {
                string legacyPluginRootPath = GetAllUtilitiesPluginPackages(rootPluginPath);
                if (legacyPluginRootPath != null)
                {
                    string legacyPluginPath = Path.Combine(legacyPluginRootPath, folderName, pluginName);
                    if (File.Exists(Path.GetFullPath(legacyPluginPath)))
                    {
                        pluginPath = legacyPluginPath;
                    }
                }
            }
#endif // OVR_INTERNAL_CODE

            return pluginPath;
        }

        private static string GetFileChecksum(string filePath)
        {
            using var md5 = new MD5CryptoServiceProvider();
            byte[] buffer = md5.ComputeHash(File.ReadAllBytes(filePath));
            byte[] pathBuffer = md5.ComputeHash(Encoding.UTF8.GetBytes(filePath));
            return string.Join(null, buffer.Select(b => b.ToString("x2"))) + string.Join(null, pathBuffer.Select(b => b.ToString("x2")));
        }

        private static void RestartUnityEditor()
        {
            if (_unityRunningInBatchMode)
            {
                Debug.LogWarning("Restarting editor is not supported in batch mode");
                return;
            }

            SetIsRestartPending(true);
            EditorApplication.OpenProject(GetCurrentProjectPath());
        }

        private static string GetCurrentProjectPath()
        {
            DirectoryInfo projectPath = Directory.GetParent(Application.dataPath);
            if (projectPath == null)
            {
                throw new DirectoryNotFoundException("Unable to find project path.");
            }
            return projectPath.FullName;
        }

        private static void ConfigurePlugin(PluginImporter plugin, PluginPlatform platform)
        {
            plugin.SetCompatibleWithEditor(false);
            plugin.SetCompatibleWithAnyPlatform(false);
            plugin.SetCompatibleWithPlatform(BuildTarget.Android, false);
            plugin.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows, false);
            plugin.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows64, false);
            plugin.SetCompatibleWithPlatform(BuildTarget.StandaloneOSX, false);

            switch (platform)
            {
                case PluginPlatform.AndroidOpenXR:
                    plugin.SetCompatibleWithPlatform(BuildTarget.Android, true);
                    break;
                case PluginPlatform.Win64OpenXR:
                    plugin.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows64, true);
                    plugin.SetCompatibleWithEditor(true);
                    plugin.SetEditorData("CPU", "X86_64");
                    plugin.SetEditorData("OS", "Windows");
                    plugin.SetPlatformData("Editor", "CPU", "X86_64");
                    plugin.SetPlatformData("Editor", "OS", "Windows");
                    break;
                default:
                    throw new ArgumentException("Unsupported BuildTarget: " + platform);
            }

            plugin.SaveAndReimport();
        }

        private static bool GetIsRestartPending()
        {
            return SessionState.GetBool(_isRestartPendingKey, false);
        }

        private static void SetIsRestartPending(bool isRestartPending)
        {
            SessionState.SetBool(_isRestartPendingKey, isRestartPending);
        }

        private static bool GetHasRunOnce()
        {
            return SessionState.GetBool(_hasRunOnceKey, false);
        }

        private static void SetHasRunOnce(bool hasRunOnce)
        {
            SessionState.SetBool(_hasRunOnceKey, hasRunOnce);
        }

#if OVR_INTERNAL_CODE
        private static readonly Version invalidVersion = new("0.0.0");
        private static readonly string disabledSuffix = @".disabled";

        private static string GetAllUtilitiesPluginPackages(string pluginRootPath)
        {
            string latestDir = null;
            Version latestVersion = new("0.0.0");
            if (Directory.Exists(pluginRootPath))
            {
                var dirs = Directory.GetDirectories(pluginRootPath);
                foreach (string dir in dirs)
                {
                    Version version = GetPluginVersion(dir);
                    if (version > latestVersion)
                    {
                        latestVersion = version;
                        latestDir = dir;
                    }
                }
            }

            if (latestDir == null)
            {
                return null;
            }
            return Path.Combine(pluginRootPath, Path.GetFileName(latestDir));
        }

        private static Version GetPluginVersion(string path)
        {
            Version pluginVersion;
            try
            {
                pluginVersion = new Version(Path.GetFileName(path));
            }
            catch
            {
                pluginVersion = invalidVersion;
            }

            if (pluginVersion == invalidVersion)
            {
                // Unable to determine version from path, fallback to Win64 DLL meta data
                path = Path.Combine(path, "Win64OpenXR", "OVRPlugin.dll");
                if (!File.Exists(path))
                {
                    path += disabledSuffix;
                    if (!File.Exists(path))
                    {
                        return invalidVersion;
                    }
                }
                System.Diagnostics.FileVersionInfo pluginVersionInfo = System.Diagnostics.FileVersionInfo.GetVersionInfo(path);
                if (pluginVersionInfo == null || pluginVersionInfo.ProductVersion == null ||
                    pluginVersionInfo.ProductVersion == "")
                {
                    return invalidVersion;
                }
                pluginVersion = new Version(pluginVersionInfo.ProductVersion);
            }

            return pluginVersion;
        }
#endif // OVR_INTERNAL_CODE
    }
}
