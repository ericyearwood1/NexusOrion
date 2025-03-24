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
using System.Diagnostics;
using System.Threading.Tasks;
using System;
using System.IO;

static class InternalTools
{
    // By default, workingDirectory uses the project's path. It can be overridden for
    // cases like using fbsource UPM packages for a project that lives outside fbsource.
    public static async Task<string> GetFbSource(string workingDirectory = null)
    {
#if UNITY_EDITOR_OSX
        return FindHgFolder();
#else
        return await RunProcess("hg", "root", workingDirectory);
#endif
    }

    public static string FindHgFolder()
    {
        string currentDirectory = Environment.CurrentDirectory;


        while (!Directory.Exists(Path.Combine(currentDirectory, ".hg")))
        {
            currentDirectory = Directory.GetParent(currentDirectory)?.FullName;
        }

        if(Directory.Exists(Path.Combine(currentDirectory, ".hg"))) {
            return currentDirectory;
        } else
        {
            return null;
        }
    }

    public static async Task<string> RunProcess(string command, string arguments, string workingDirectory = null)
    {
#if !UNITY_EDITOR_WIN
        UnityEngine.Debug.Log("A new process was executed");
        UnityEngine.Debug.Log("Command: "+ command);
        UnityEngine.Debug.Log("Arguments: " + arguments);
#endif

        var processStartInfo = new ProcessStartInfo
        {
            FileName = command,
            Arguments = arguments,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            WindowStyle = ProcessWindowStyle.Hidden,
            CreateNoWindow = true,
        };
        if (workingDirectory != null)
        {
            processStartInfo.WorkingDirectory = Path.GetFullPath(workingDirectory);
        }

        using var process = new Process
        {
            StartInfo = processStartInfo
        };

        process.Start();
        var stdout = await process.StandardOutput.ReadToEndAsync();
        process.WaitForExit();
        var result = stdout.Trim();
#if !UNITY_EDITOR_WIN
        UnityEngine.Debug.Log("Result: " + result);
#endif
        return result;
    }
}
#endif
