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

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

#if OVRPLUGIN_TESTING

class OVRScenePlayModeTests : AnchorTestFixture
{
    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        CreateDefaultCameraRig();
        yield return new WaitUntil(() => OVRManager.OVRManagerinitialized);
    }

    IEnumerator RequestSpaceSetup(string labels, OVRPlugin.Result expectedResult = OVRPlugin.Result.Success)
    {
        yield return WaitForSpaceSetup(OVRScene.RequestSpaceSetup(labels), labels, expectedResult);
    }

    IEnumerator WaitForSpaceSetup(OVRTask<bool> task, string expectedLabels, OVRPlugin.Result expectedResult)
    {
        // Find the request
        var (requestId, request) = SceneCaptureRequests
            .First(kvp => OVRTask.GetId(kvp.Key) == task._id);

        if (string.IsNullOrEmpty(request.Value))
        {
            Assert.That(string.IsNullOrEmpty(expectedLabels));
        }
        else
        {
            Assert.That(request.Value, Is.EqualTo(expectedLabels));
        }

        HandleSceneCaptureRequest(new OVRDeserialize.SceneCaptureCompleteData
        {
            RequestId = requestId,
            Result = (int)expectedResult
        });

        yield return new WaitUntil(() => task.IsCompleted);

        Assert.That(task.GetResult(), Is.EqualTo((int)expectedResult >= 0));
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator RequestSpaceSetupSucceedsWithNullLabelsString()
    {
        yield return RequestSpaceSetup(null);
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator RequestSpaceSetupSucceedsWithCommaSeparatedList()
    {
        yield return RequestSpaceSetup("TABLE,TABLE");
    }

    [Test]
    public void RequestSpaceSetupThrowsWithInvalidStringLabel() => Assert.Throws<ArgumentException>(() =>
        OVRScene.RequestSpaceSetup("TABLE,InvalidLabel,WALL"));
}

#endif
