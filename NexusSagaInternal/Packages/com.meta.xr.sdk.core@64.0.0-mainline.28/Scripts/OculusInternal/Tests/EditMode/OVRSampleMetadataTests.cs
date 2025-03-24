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
using Meta.XR.Samples;
using Meta.XR.Samples.Telemetry;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

internal class OVRSampleMetadataTests : OVRPluginEditModeTest
{
    private string _testSceneName;
    private GameObject _sampleMetaDataObject;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        var activeScene = SceneManager.GetActiveScene();
        _testSceneName = activeScene.name;
        yield return base.UnitySetUp();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        yield return base.UnityTearDown();
    }


    [UnityTest]
    public IEnumerator TestOpenAndClose()
    {
        var gameObject = new GameObject();
        gameObject.AddComponent<SampleMetadata>();
        OVRTelemetry.Expect(SampleTelemetryEvents.EventTypes.Open)
            .AddAnnotation(SampleTelemetryEvents.AnnotationTypes.Sample, _testSceneName);
        yield return null;

        Object.DestroyImmediate(gameObject);
        OVRTelemetry.Expect(SampleTelemetryEvents.EventTypes.Close)
            .AddAnnotation(SampleTelemetryEvents.AnnotationTypes.Sample, _testSceneName);
        yield return null;

        OVRTelemetry.TestExpectations();
        yield return null;
    }
}

#endif // OVRPLUGIN_TESTING
