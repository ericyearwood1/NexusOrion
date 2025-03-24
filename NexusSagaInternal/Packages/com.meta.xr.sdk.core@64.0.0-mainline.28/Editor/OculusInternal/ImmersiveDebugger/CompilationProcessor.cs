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
using UnityEditor;
using UnityEditor.Compilation;
using Meta.XR.Editor.Callbacks;
using Meta.XR.ImmersiveDebugger.DebugData;
using System.IO;
using System.Linq;
using System.Reflection;
using UnityEngine;
using Assembly = System.Reflection.Assembly;

namespace Meta.XR.ImmersiveDebugger.Editor
{
    /// <summary>
    /// If ImmersiveDebugger is enabled,
    /// listen for assembly compilation events and bake OVRDebugMember
    /// requests from those changed assembly into ScriptableObject.
    /// Only runs in Editor and during compilation time.
    /// </summary>
    internal class CompilationProcessor
    {
        [InitializeOnLoadMethod]
        private static void OnLoad() {
            InitializeOnLoad.Register(Init);
        }

        private static void Init()
        {
            if (RuntimeSettings.Instance.ImmersiveDebuggerEnabled)
            {
                // Only add debug types incrementally when ImmersiveDebugger is enabled
                CompilationPipeline.assemblyCompilationFinished += (s, _) => OnAssemblyCompilationEnded(s);
            }
        }

        internal static void OnAssemblyCompilationEnded(string s)
        {
            if (BuildPipeline.isBuildingPlayer || string.IsNullOrEmpty(s))
            {
                // During build time we don't get assemblies with metadata and we can't create assets.
                return;
            }
            string absolutePath = Path.Combine(Application.dataPath, "..", s);
            // not try-catch here because any compiler error would be intercepted as not surfaced well in console
            Assembly assembly = Assembly.LoadFile(absolutePath);
            var types = assembly.GetTypes().Where(
                t => t.GetMembers(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static | BindingFlags.Instance).Any(
                    m => m.GetCustomAttribute<DebugMember>() != null));
            RuntimeSettings.UpdateTypes(assembly.GetName().Name, types.ToList().ConvertAll(type => type.Name));
        }
    }
}

#endif
