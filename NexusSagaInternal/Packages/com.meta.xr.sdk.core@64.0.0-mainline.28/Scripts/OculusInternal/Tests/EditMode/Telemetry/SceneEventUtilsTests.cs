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

#if OVRPLUGIN_TESTING

using System.Collections;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.TestTools;

namespace Meta.XR.Samples.Telemetry
{
    internal class SceneEventUtilsTests : OVRPluginEditModeTest
    {
        private const string path = "Assets/Test.unity";

        [UnitySetUp]
        public override IEnumerator UnitySetUp()
        {
            yield return base.UnitySetUp();

            var test = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene);
            EditorSceneManager.SaveScene(test, path);
        }

        [UnityTearDown]
        public override IEnumerator UnityTearDown()
        {
            AssetDatabase.DeleteAsset(path);
            yield return base.UnityTearDown();
        }

        [UnityTest]
        public IEnumerator SceneOpenAndCloseSendsEvent()
        {
            var guid = AssetDatabase.AssetPathToGUID(path);
            var scene = EditorSceneManager.OpenScene(path);
            OVRTelemetry.Expect(SampleTelemetryEvents.EventTypes.SceneOpen)
                .AddAnnotation(SampleTelemetryEvents.AnnotationTypes.Guid, guid);
            yield return null;
            EditorSceneManager.CloseScene(scene, true);
            OVRTelemetry.Expect(SampleTelemetryEvents.EventTypes.SceneClose)
                .AddAnnotation(SampleTelemetryEvents.AnnotationTypes.Guid, guid);
            yield return null;
        }
    }
}


#endif // OVRPLUGIN_TESTING
