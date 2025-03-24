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

using UnityEngine;
using UnityEngine.SceneManagement;

#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif

namespace Meta.XR.Samples.Telemetry
{
    public static class SceneEventUtils
    {
#if UNITY_EDITOR
        [InitializeOnLoadMethod]
        private static void Initialize()
        {
            EditorSceneManager.sceneOpened += OnSceneOpened;
#if UNITY_2022_2_OR_NEWER
            EditorSceneManager.sceneManagerSetupRestored += OnScenesRestored;
#endif
            EditorSceneManager.sceneClosed += OnSceneClosed;
        }

        private static void OnScenesRestored(Scene[] scenes)
        {
            foreach (var scene in scenes)
            {
                SendEvent(scene, SampleTelemetryEvents.EventTypes.SceneOpen);
            }
        }

        private static void OnSceneOpened(Scene scene, OpenSceneMode mode)
        {
            SendEvent(scene, SampleTelemetryEvents.EventTypes.SceneOpen);
        }

        private static void OnSceneClosed(Scene scene)
        {
            SendEvent(scene, SampleTelemetryEvents.EventTypes.SceneClose);
        }

        private static void SendEvent(Scene scene, int eventType)
        {
            var guid = AssetDatabase.AssetPathToGUID(scene.path);
            if (string.IsNullOrEmpty(guid)) return;

            OVRTelemetry.Start(eventType)
                .AddAnnotation(SampleTelemetryEvents.AnnotationTypes.Guid,
                    guid)
                .AddAnnotation(SampleTelemetryEvents.AnnotationTypes.BuildTarget,
                    EditorUserBuildSettings.selectedBuildTargetGroup.ToString())
                .AddAnnotation(SampleTelemetryEvents.AnnotationTypes.RuntimePlatform,
                    Application.platform.ToString())
                .AddAnnotation(SampleTelemetryEvents.AnnotationTypes.InEditor,
                    Application.isEditor.ToString())
                .Send();
        }
#endif
    }
}
