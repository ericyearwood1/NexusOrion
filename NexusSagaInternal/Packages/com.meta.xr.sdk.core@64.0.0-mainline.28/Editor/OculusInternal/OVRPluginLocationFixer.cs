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

using Oculus.VR.Editor;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;
using Debug = UnityEngine.Debug;

public class OVRPluginLocationFixer : EditorWindow
{
    private const string MenuTitle = "Fix OVRPlugin Location";
    private const string Menu = "Oculus/Internal/" + MenuTitle;
    private const string WindowTitle = "OVRPlugin Location Fixer";

    [MenuItem(Menu)]
    public static void FixOVRPluginLocation()
    {
        if (!OVRPluginInfo.IsCoreSDKModifiable())
        {
            return;
        }

        string errorMessage = "";
        string moveMessage = "";
        Dictionary<string, string> needsFixing = new();
        string targetBasePluginPath = OVRPluginInfo.GetPluginRootPath();

        foreach (var pluginInfo in OVRPluginInfoOpenXR._pluginInfos)
        {
            string targetPluginPath = Path.GetFullPath(Path.Combine(targetBasePluginPath, pluginInfo.Value.folderName, pluginInfo.Value.pluginName));
            string[] existingPluginPaths = GetCustomOVRPluginLocations(pluginInfo.Key);
            if (existingPluginPaths.Length > 1)
            {
                errorMessage = errorMessage + pluginInfo.Key + " has multiple existing OVRPlugins. Please delete all except one. Paths: " +
                    string.Join(", ", existingPluginPaths) + "\n\n";
            }
            else if (existingPluginPaths.Length == 0)

            {
                moveMessage = moveMessage + "No custom OVRPlugins found for platform " + pluginInfo.Key + "\n\n";
            }
            else

            {
                if (targetPluginPath.Equals(existingPluginPaths[0]))
                {

                    moveMessage = moveMessage + pluginInfo.Key + " OVRPlugin is already in the right place.\n\n";
                }
                else
                {
                    needsFixing.Add(existingPluginPaths[0], targetPluginPath);
                    moveMessage = moveMessage + pluginInfo.Key + " plugin will be moved from " + existingPluginPaths[0] + " to " + targetPluginPath + ".\n\n";
                }
            }
        }

        if (!string.IsNullOrEmpty(errorMessage))
        {
            EditorUtility.DisplayDialog(WindowTitle, "OVRPlugin issues found: \n\n" + errorMessage, "Ok");
            Debug.LogWarning(errorMessage);
        }
        else if (needsFixing.Count == 0)
        {
            EditorUtility.DisplayDialog(WindowTitle, moveMessage, "Ok");
        }
        else
        {
            if (EditorUtility.DisplayDialog(WindowTitle, moveMessage + "Proceed with fixing?", "Yes", "No"))
            {
                foreach (var item in needsFixing)
                {
                    MovePlugin(item.Key, item.Value);
                }
                AssetDatabase.Refresh(ImportAssetOptions.ForceUpdate);
                EditorUtility.RequestScriptReload();
            }
            else
            {
                Debug.LogWarning("OVRPlugin is currently in the wrong location, and may fail to work with internal " +
                    "OVRPlugin build and upgrade scripts. To fix this, go to Oculus -> Internal -> Fix OVRPlugin Location.");
            }
        }
    }

    private static string[] GetCustomOVRPluginLocations(PluginPlatform platform)
    {
        string regex = platform switch
        {
            PluginPlatform.AndroidOpenXR => @"OVRPlugin\.aar$",
            PluginPlatform.Win64OpenXR => @"OVRPlugin\.dll$",
            _ => throw new System.ArgumentException("Unsupported BuildTarget: " + platform),
        };
        return PluginImporter.GetAllImporters().Where(p =>
            Regex.IsMatch(p.assetPath, regex) && !p.assetPath.Contains("com.unity.xr.oculus")
        ).ToArray().Select(p => Path.GetFullPath(p.assetPath)).ToArray();
    }

    private static void MovePlugin(string currentPath, string targetPath)
    {
        new FileInfo(targetPath).Directory.Create();
        File.Move(currentPath, targetPath);
        if (File.Exists(currentPath + ".meta"))
        {
            File.Move(currentPath + ".meta", targetPath + ".meta");
        }
    }
}

#endif  // OVR_INTERNAL_CODE
