// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using TinyJson;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace ARGlasses.Interaction
{
    public static class ExternalProcess
    {
        public static string CombineFull(string s1, string s2) => Path.GetFullPath(Path.Combine(s1, s2));
        public static string RepoPackagesRoot => CombineFull(ProjectRoot, "../../packages");
        public static string GetRepoPackageFolder(string package) => CombineFull(RepoPackagesRoot, package);

        public static string ProjectRoot => CombineFull(Application.dataPath, "..");
        public static string ProjectRelativePath(string relativePath) => CombineFull(ProjectRoot, relativePath);
        public static string BuildsFolder => CombineFull(ProjectRoot, "Builds");

        public static string AssetsRoot => Application.dataPath;
        public static string PackagesRoot => CombineFull(ProjectRoot, "Packages");
        public static string ManifestJson => CombineFull(PackagesRoot, "manifest.json");
        public static string LauncherPackageRoot => CombineFull(PackagesRoot, "com.ards.interaction.subprocess");
        public static string BatFolder => CombineFull(LauncherPackageRoot, "Bat");
        public static string BatFilename => "runCTRLR.bat";

        public static string ManifestPath =>
            Path.Combine(Path.GetDirectoryName(Application.dataPath), "Packages", "manifest.json");

        public static Dictionary<string, object> Manifest =>
            File.ReadAllText(ManifestPath).FromJson<Dictionary<string, object>>();

        public static IEnumerable<string> FindFilesByName(string fileName)
        {
            var ext = Path.GetExtension(fileName);
            var filter = Path.GetFileNameWithoutExtension(fileName);
            return AssetDatabase.FindAssets(filter).Select(AssetDatabase.GUIDToAssetPath)
                .Where(path => path.EndsWith(ext));
        }

        public static void RunCmd(IEnumerable<string> lines, string workingDirectory = null, bool admin = false)
        {
            // return RunCmd(string.Join(" && ", lines), workingDirectory, admin);
            foreach (var line in lines)
            {
                var process = RunCmd(line, workingDirectory, admin);
                process.WaitForExit();
            }
        }

        public static Process RunCmd(string commandString, string workingDirectory = null, bool admin = false)
        {
            var isMac = Application.platform == RuntimePlatform.OSXEditor;
            var noWindow = true;

            var fileName = isMac ? "/bin/zsh" : "cmd.exe";
            var arguments = isMac
                ? $"-c '{commandString.Replace("maui", "/opt/facebook/bin/maui").Replace("pause", "echo done")}'"
                : $"/C {commandString}";
            var process = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    WindowStyle = ProcessWindowStyle.Normal,
                    FileName = fileName,
                    Arguments = arguments,
                    Verb = admin ? "runas" : null,
                    CreateNoWindow = noWindow,
                    UseShellExecute = !noWindow,
                    RedirectStandardOutput = noWindow,
                    RedirectStandardError = noWindow,
                    WorkingDirectory = workingDirectory,
                }
            };
            process.Start();
            var output = process.StandardOutput.ReadToEndAsync();
            var errors = process.StandardError.ReadToEndAsync();
            output.ContinueWith(s => Debug.Log(s.Result));
            errors.ContinueWith(s =>
            {
                if (!string.IsNullOrEmpty(s.Result)) Debug.LogError(s.Result);
            });
            // process.WaitForExit();
            return process;
        }

        public static Process RunFile(string filename, string directory, bool hidden = false)
        {
            return Start(filename, directory, hidden, useShellExecute: true);
        }

        public static Process RunExe(string exeNameWithExtension, string workingDirectory, bool hidden = false,
            string arguments = "")
        {
            var environmentPath = Environment.GetEnvironmentVariable("PATH");
            Dictionary<string, string> environmentVariables = new Dictionary<string, string>
            {
                // { "PYTHONPATH", python },
                // { "CONDA_PREFIX", condaPrefix },
                // { "PATH", $"{python};{environmentPath}" },
                // { "RTECH_PATH", rTechPath }
            };
            return Start(exeNameWithExtension, workingDirectory, hidden, arguments, environmentVariables, false);
        }

        public static Process Start(string executableNameWithExtension, string workingDirectory, bool hidden,
            string arguments = "", Dictionary<string, string> environmentVariables = null, bool useShellExecute = true,
            string absoluteFileName = null, bool tryCloseExistingProcess = false)
        {
            try
            {
                if (tryCloseExistingProcess) Stop(executableNameWithExtension);

                if (string.IsNullOrEmpty(absoluteFileName))
                    absoluteFileName = Path.Combine(workingDirectory, $"{executableNameWithExtension}");
                var logConsole = !useShellExecute; // useShellExecute cannot redirect output

                var startInfo = new ProcessStartInfo
                {
                    WindowStyle = hidden ? ProcessWindowStyle.Hidden : ProcessWindowStyle.Normal,
                    UseShellExecute = useShellExecute,
                    CreateNoWindow = hidden,
                    RedirectStandardOutput = logConsole,
                    RedirectStandardError = logConsole,
                    WorkingDirectory = workingDirectory,
                    FileName = absoluteFileName,
                    Arguments = arguments,
                };

                if (environmentVariables != null)
                {
                    foreach (string key in environmentVariables.Keys)
                        startInfo.EnvironmentVariables[key] = environmentVariables[key];
                    var envVarString = environmentVariables.Select((k, v) => $"{k}: {v}");
                    Debug.Log($"Using EnvironmentVariables: \n{string.Join("\n", envVarString)}");
                }

                var process = new Process { StartInfo = startInfo, EnableRaisingEvents = true, };

                if (logConsole)
                {
                    process.OutputDataReceived += (sender, e) =>
                    {
                        if (e.Data != null) Debug.Log($"{executableNameWithExtension}: {e.Data}");
                    };
                    process.ErrorDataReceived += (sender, e) =>
                    {
                        if (e.Data != null) Debug.Log($"{executableNameWithExtension}: {e.Data}");
                    };
                }

                process.Start();
                if (logConsole)
                {
                    process.BeginOutputReadLine();
                    process.BeginErrorReadLine();
                }

                return process;
            }
            catch (Exception e)
            {
                Debug.LogError(
                    $"Unable to launch {executableNameWithExtension} from {workingDirectory} with {arguments}: " +
                    e.Message);
                return null;
            }
        }


        public static void Minimize(string processName)
        {
            foreach (var existingProcess in Process.GetProcessesByName(processName))
            {
                var processWindows = ProcessUtils.GetProcessWindows(existingProcess.Id);
                foreach (var processWindow in processWindows)
                {
                    Debug.LogWarning($"Minimizing {processName} {processWindow}");
                    ProcessUtils.ShowWindow(processWindow, 11);
                }
            }
        }

        public static void Stop(string processName)
        {
            foreach (var existingProcess in Process.GetProcessesByName(processName)) Stop(existingProcess);
        }

        public static void Stop(Process process)
        {
            if (process == null || process is { HasExited: true }) return;
            Debug.Log($"Closing Process: {process.ProcessName}");
            process.CloseMainWindow();
            process.Close();
        }

        private class ProcessUtils
        {
            [DllImport("user32.dll")]
            [return: MarshalAs(UnmanagedType.Bool)]
            public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);

            [DllImport("user32.dll")]
            public static extern IntPtr FindWindowEx(IntPtr parentWindow, IntPtr previousChildWindow,
                string windowClass,
                string windowTitle);

            [DllImport("user32.dll")]
            public static extern IntPtr GetWindowThreadProcessId(IntPtr window, out int process);

            public static List<IntPtr> GetProcessWindows(int process)
            {
                List<IntPtr> foundProcessWindows = new List<IntPtr>();
                var pLast = IntPtr.Zero;
                do
                {
                    pLast = FindWindowEx(IntPtr.Zero, pLast, null, null);
                    GetWindowThreadProcessId(pLast, out var iProcess);
                    if (iProcess == process) foundProcessWindows.Add(pLast);
                } while (pLast != IntPtr.Zero);

                return foundProcessWindows;
            }
        }

        public static Process RunPyFile(string pyFileAbsolutePath, List<string> args = null, bool spawnTerminal = true)
        {
            if (!File.Exists(pyFileAbsolutePath))
            {
                Debug.LogError("Cannot find " + pyFileAbsolutePath);
                return null;
            }

            var workingDir = Directory.GetParent(pyFileAbsolutePath).FullName;
            var arguments = Path.GetFileName(pyFileAbsolutePath);
            if (args != null) arguments += $" {string.Join(" ", args)}";
            var mac = SystemInfo.operatingSystemFamily == OperatingSystemFamily.MacOSX;
            var exeName = mac ? "python" : "python.exe";
            var absoluteFileName = mac ? "/usr/bin/python3" : "C:\\Python38\\python.exe";
            return Start(exeName, workingDir, hidden: !spawnTerminal, arguments: arguments,
                absoluteFileName: absoluteFileName, useShellExecute: spawnTerminal);
        }
    }
}
