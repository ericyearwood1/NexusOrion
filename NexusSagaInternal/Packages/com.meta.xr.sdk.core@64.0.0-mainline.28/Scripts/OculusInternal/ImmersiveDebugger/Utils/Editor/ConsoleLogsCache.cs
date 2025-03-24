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
using System.Threading;
using UnityEngine;
using Meta.XR.ImmersiveDebugger.DebugData;

#if UNITY_EDITOR
using Meta.XR.Editor.Callbacks;
using UnityEditor;
#endif // UNITY_EDITOR

namespace Meta.XR.ImmersiveDebugger.Utils
{
    /// <summary>
    /// To handle the case where debugger console starts up too late, we move the logs cache earlier when assemblies loaded,
    /// and the startup logs will be cached here for consuming in console later.
    /// This class can also serve as logs caching when console is disabled then enabled later.
    /// </summary>
    internal static class ConsoleLogsCache
    {
        internal static Action<string, string, LogType> OnLogReceived;
        private static readonly List<(string, string, LogType)> StartupLogs = new List<(string, string, LogType)>();
        private static SynchronizationContext _mainThreadContext;
#if UNITY_EDITOR
        [InitializeOnLoadMethod]
#else
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterAssembliesLoaded)]
#endif
        private static void OnLoad()
        {
#if UNITY_EDITOR
            if (EditorApplication.isPlaying)
            {
                InitializeOnLoad.Register(StartCachingLogs);
            }
#else
            StartCachingLogs();
#endif
        }

        private static void StartCachingLogs()
        {
            _mainThreadContext = SynchronizationContext.Current;
            if (RuntimeSettings.Instance.ImmersiveDebuggerEnabled)
            {
                Application.logMessageReceivedThreaded += EnqueueLogEntry;
                Application.quitting += () => Application.logMessageReceivedThreaded -= EnqueueLogEntry;
            }
        }

        internal static void ConsumeStartupLogs(Action<string, string, LogType> logProcessor)
        {
            foreach (var log in StartupLogs)
            {
                logProcessor(log.Item1, log.Item2, log.Item3);
            }
            StartupLogs.Clear();
        }

        private static void EnqueueLogEntry(string logString, string stackTrace, LogType type)
        {
            _mainThreadContext.Post(_ =>
            {
                if (OnLogReceived == null)
                {
                    StartupLogs.Add((logString, stackTrace, type));
                }
                else
                {
                    // We only cache startup logs if no console receiving them
                    OnLogReceived.Invoke(logString, stackTrace, type);
                }

            },null);
        }
    }
}

#endif
