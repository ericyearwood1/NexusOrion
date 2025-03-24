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

using System.IO;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;

class SampleFrameworkLinker
{
    [MenuItem("Oculus/Internal/Symlink Samples + Platform")]
    public static async Task LinkSampleFramework()
    {
        var fbSource = await InternalTools.GetFbSource();

        if (Directory.Exists(Path.Combine("Assets", "Oculus", "SampleFramework")))
        {
            Debug.LogWarning($"Oculus/SampleFramework already exists. Skipping.");
        }
        else
        {
            await CreateSymlink("SampleFramework",
                Path.Combine("Assets", "Oculus"),
                Path.Combine(fbSource, "arvr", "projects", "integrations", "UnitySampleFramework", "Assets", "Oculus"));
        }

        if (Directory.Exists(Path.Combine("Assets", "Oculus", "Platform")))
        {
            Debug.LogWarning($"Oculus/Platform already exists. Skipping Platform SDK.");
        }
        else
        {
            await CreateSymlink("Platform",
                Path.Combine("Assets", "Oculus"),
                Path.Combine(fbSource, "arvr", "projects", "platform-sdk", "1stParty", "LibOVRPlatform", "Unity",
                    "Assets", "Oculus"));
        }

        AssetDatabase.Refresh();
    }

    static async Task CreateSymlink(string linkedDirectoryName, string sourceDir, string destDir)
    {
#if UNITY_EDITOR_WIN
        var source = Path.Combine(sourceDir, linkedDirectoryName);
        var dest = Path.Combine(destDir, linkedDirectoryName);
        await InternalTools.RunProcess("cmd.exe", $"/c mklink /d /j {source} {dest}");
        Debug.Log($"Linked {source} to {dest}");
#else
        await InternalTools.RunProcess("ln", "-s " + Path.Combine(destDir, linkedDirectoryName) + " " + sourceDir);
#endif
    }
}

#endif // OVR_INTERNAL_CODE
