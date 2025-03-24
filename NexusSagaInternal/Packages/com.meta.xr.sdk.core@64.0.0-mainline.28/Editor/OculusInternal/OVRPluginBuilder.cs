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
using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

class OVRPluginBuilder : EditorWindow
{
    enum CodeGeneration
    {
        Debug,
        Release,
    }

    enum Flavor
    {
        OpenXR,
        VRCamera,
#if !UNITY_EDITOR_OSX
        Legacy,
#endif
    }

    enum Target
    {
        Android,
#if UNITY_EDITOR_WIN
        Windows,
#endif
#if UNITY_EDITOR_OSX
        MacOsARM,
#endif
    }

    Target _target;

    Flavor _flavor;

    CodeGeneration _codeGeneration;

    bool _internal =
#if UNITY_EDITOR_WIN || UNITY_EDITOR_OSX
        true;
#else
        false;
#endif

    Vector2 _scrollPosition;

    StringBuilder _output = new StringBuilder();

    bool _busy;

    string _version;

    static string _pluginsRootPath;

    struct BusyScope : IDisposable
    {
        OVRPluginBuilder _builder;

        public BusyScope(OVRPluginBuilder builder)
        {
            _builder = builder;
            _builder._busy = true;
        }

        public void Dispose() => _builder._busy = false;
    }

    [MenuItem("Oculus/Internal/OVRPlugin Builder")]
    public static void ShowWindow()
    {
        GetWindow<OVRPluginBuilder>();
    }

    static bool TryGetMatch(string input, string pattern, out int value)
    {
        var match = Regex.Match(input, pattern);
        value = match.Success
            ? int.Parse(match.Groups[1].Captures[0].Value)
            : default;
        return match.Success;
    }

    void OnEnable()
    {
        titleContent = new GUIContent($"OVRPlugin Builder");
        _pluginsRootPath = OVRPluginInfo.GetPluginRootPath();
        Task.Run(async () =>
        {
            var path = Path.Combine(await InternalTools.GetFbSource(_pluginsRootPath), "arvr", "projects", "integrations", "OVRPlugin",
                "Src", "OVR_Plugin_Types.h");
            var contents = File.ReadAllText(path);
            TryGetMatch(contents, "#define OVRP_MAJOR_VERSION ([0-9]+)", out var major);
            TryGetMatch(contents, "#define OVRP_MINOR_VERSION ([0-9]+)", out var minor);
            TryGetMatch(contents, "#define OVRP_PATCH_VERSION ([0-9]+)", out var patch);
            if (string.IsNullOrEmpty(_version))
            {
                _version = $"{major}.{minor}.{patch}";
            }
        });
    }

#if UNITY_EDITOR_WIN
    const bool IsWindows = true;
#else
    const bool IsWindows = false;
#endif

#if UNITY_EDITOR_OSX
    const bool IsMac = true;
#else
    const bool IsMac = false;
#endif

    void OnGUI()
    {
        _codeGeneration = (CodeGeneration)EditorGUILayout.EnumPopup("Code Generation", _codeGeneration);
        _flavor = (Flavor)EditorGUILayout.EnumPopup("Flavor", _flavor);
        using (new EditorGUI.DisabledScope(!IsWindows && !IsMac))
        {
            _target = (Target)EditorGUILayout.EnumPopup("Target", _target);
            _internal = EditorGUILayout.Toggle("Internal", _internal);
        }

        _version = EditorGUILayout.TextField("Version", _version);

        if (!OVRPluginInfo.IsCoreSDKModifiable())
        {
            GUI.enabled = false;
            EditorGUILayout.HelpBox("Core SDK is imported in a non-modifiable state, so this utility is disabled.", MessageType.Warning);
        }
        else
        {
            EditorGUILayout.HelpBox("This tool now builds OVRPlugin directly to the Plugins folder without a version folder. " +

                "If you have old OVRPlugins inside Plugins/1.xx.0 folders, please delete those first before building.", MessageType.Info);
        }

        using (new EditorGUI.DisabledScope(_busy))
        {
            if (GUILayout.Button("Build"))
            {
                _pluginsRootPath = OVRPluginInfo.GetPluginRootPath();
                Task.Run(Build).ContinueWith((task) =>
                {
                    try
                    {
                        if (!task.IsFaulted)
                        {
                            AssetDatabase.Refresh(ImportAssetOptions.ForceUpdate);
                            OVRPluginInfoOpenXR.BatchmodeCheckHasPluginChanged();
                        }
                    }
                    catch (Exception e)
                    {
                        Debug.LogError(e);
                    }
                }, TaskScheduler.FromCurrentSynchronizationContext());
            }

            if (GUILayout.Button("Delete build artifacts"))
            {
                Task.Run(DeleteBuildArtifacts);
            }
        }

        EditorGUILayout.LabelField("Build Output");
        using (var scroll = new GUILayout.ScrollViewScope(_scrollPosition, GUILayout.ExpandHeight(true)))
        {
            _scrollPosition = scroll.scrollPosition;
            EditorGUILayout.TextArea(_output.ToString(), EditorStyles.textArea, GUILayout.ExpandHeight(true));
        }
    }

    async Task DeleteBuildArtifacts()
    {
        using var busy = new BusyScope(this);

        _output.Clear();
        var fbsource = await InternalTools.GetFbSource(_pluginsRootPath);
        var path = Path.Combine(fbsource, "arvr", "projects", "integrations", "OVRPlugin", "Build", "out");
        var extensions = new[] { "dll", "aar", "dylib" };

        _output.AppendLine($"Deleting all {string.Join(", ", extensions)}...");
        foreach (var ext in extensions)
        {
            foreach (var file in Directory.EnumerateFiles(path, $"*.{ext}", SearchOption.AllDirectories))
            {
                _output.AppendLine($"Deleting {file}");
                File.Delete(file);
            }
        }

        _output.AppendLine("Done");
    }

    async Task Build()
    {
        using var busy = new BusyScope(this);
        // Since the editor might have a lock on old ovrplugin files, build to a temp folder first
        var tempPluginsPath = Path.Combine(_pluginsRootPath, "temp");
        try
        {
            _output.Clear();

#if UNITY_EDITOR_OSX
            string specificPluginPath;
            if (_target == Target.MacOsARM)
            {
                specificPluginPath = Path.Combine(tempPluginsPath, "MacOpenXR");
            } else
            {
                specificPluginPath = Path.Combine(tempPluginsPath, _flavor switch
                {
                    Flavor.OpenXR => "AndroidOpenXR",
                    Flavor.VRCamera => "AndroidUniversal",
                    _ => throw new NotImplementedException($"{_flavor} not supported."),
                });
            }
#endif
            Directory.CreateDirectory(tempPluginsPath);
            _output.AppendLine($"Detecting fbsource directory...");
            var fbSource = await InternalTools.GetFbSource(_pluginsRootPath);
            _output.AppendLine($"fbsource determined to be '{fbSource}'");

#if UNITY_EDITOR_WIN
            var fileName =
                Path.Combine(fbSource, "arvr", "projects", "integrations", "OVRPlugin", "Build", "Build.bat");
            var args = string.Join(" ",
                $"-{_codeGeneration}",
                $"-{_flavor}",
                $"-{_target}",
                _internal ? "-Internal" : "",
                "-Unity",
                Path.GetFullPath(tempPluginsPath));
#else
            Directory.CreateDirectory(specificPluginPath);
            var fileName = "";
            var args = "";
            if (_target == Target.Android)
            {
                fileName = Path.Combine(fbSource, "arvr", "projects", "integrations", "OVRPlugin",
                "Build", "Android", "Build.sh");
                args = string.Join(" ",
                    _codeGeneration switch
                    {
                        CodeGeneration.Debug => _flavor switch
                        {
                            Flavor.OpenXR => "-DebugOpenXR",
                            Flavor.VRCamera => "-VRCameraDebug",
                            _ => "-Debug"
                        },
                        CodeGeneration.Release => _flavor switch
                        {
                            Flavor.OpenXR => "-ReleaseOpenXR",
                            Flavor.VRCamera => "-VRCameraRelease",
                            _ => "-Release"
                        },
                        _ => ""
                    },
                    $"-Unity {Path.GetFullPath(specificPluginPath)}");
            }
            else if(_target == Target.MacOsARM)
            {
                fileName = Path.Combine(fbSource, "arvr", "projects", "integrations", "OVRPlugin",
                "Build", "Mac", "build_macos.sh");
                args = string.Join(" ",
                    _internal ? "-internal" : "-public",
                    $"-Unity {Path.GetFullPath(specificPluginPath)}");
            } else
            {
                throw new NotImplementedException($"{_target} not supported.");
            }
#endif

#if UNITY_EDITOR_OSX
            string path = Environment.GetEnvironmentVariable("PATH");
            path += ":/usr/local/bin";
            path += ":/opt/facebook/hg/bin/";
#endif

            var startInfo = new ProcessStartInfo
            {
                WorkingDirectory = fbSource,
                FileName = fileName,
                Arguments = args,
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                WindowStyle = ProcessWindowStyle.Hidden,
#if UNITY_EDITOR_OSX
                EnvironmentVariables = { ["PATH"] = path },
#endif
                CreateNoWindow = true
            };

            _output.AppendLine($"Executing command:");
            _output.AppendLine($"{startInfo.FileName} {startInfo.Arguments}");

            using var process = new Process
            {
                StartInfo = startInfo
            };

            process.ErrorDataReceived += (sender, args) =>
            {
                if (string.IsNullOrEmpty(args.Data)) return;
                _output.AppendLine(args.Data);
            };

            process.OutputDataReceived += (sender, args) =>
            {
                if (string.IsNullOrEmpty(args.Data)) return;
                _output.AppendLine(args.Data);
            };

            process.Start();
            process.BeginErrorReadLine();
            process.BeginOutputReadLine();

            await Task.Run(() => process.WaitForExit());

            _output.AppendLine();
            _output.AppendLine($"Exited with code {process.ExitCode}.");

            // Rename old OVRPlugins, then move new ones from temp to the right location
            UpdateOVRPluginFiles(tempPluginsPath, _pluginsRootPath);
            if (Directory.Exists(tempPluginsPath))
            {
                Directory.Delete(tempPluginsPath, true);
            }
        }
        catch (Exception e)
        {
            Debug.LogError(e);
            _output.AppendLine(e.ToString());
            if (Directory.Exists(tempPluginsPath))
            {
                Directory.Delete(tempPluginsPath, true);
            }
            throw;
        }
    }

    private static void UpdateOVRPluginFiles(string tempRootPath, string targetRootPath)
    {
        var pluginFolders = Directory.GetDirectories(tempRootPath);
        foreach (string pluginFolder in pluginFolders)
        {
            string folderName = new DirectoryInfo(pluginFolder).Name;
            var newPluginFiles = Directory.GetFiles(pluginFolder);
            foreach (string newPluginFile in newPluginFiles)
            {
                string fileName = Path.GetFileName(newPluginFile);
                string targetFolderPath = Path.Combine(targetRootPath, folderName);
                string targetFilePath = Path.Combine(targetFolderPath, fileName);
                if (File.Exists(targetFilePath))
                {
                    if (File.Exists(targetFilePath + ".old"))
                    {
                        File.Delete(targetFilePath + ".old");
                    }
                    File.Move(targetFilePath, targetFilePath + ".old");
                }
                else if (!Directory.Exists(targetFolderPath))
                {
                    Directory.CreateDirectory(targetFolderPath);
                }

                File.Move(newPluginFile, targetFilePath);
            }
        }
    }
}

#endif // OVR_INTERNAL_CODE
