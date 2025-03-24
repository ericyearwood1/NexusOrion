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

using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Meta.XR.BuildingBlocks.Editor;
using NUnit.Framework;
using UnityEngine;

#if OVRPLUGIN_TESTING

public class CameraRigBuildingBlocksTest : BuildingBlocksTest
{
    public override string BlockId => BlockDataIds.CameraRig;
    private SceneManagerTests _sceneManagerTests;

    public override IEnumerator Setup(TestContext testContext)
    {
        _sceneManagerTests = testContext.SceneManagerTestFixture;

        while (!OVRManager.OVRManagerinitialized)
        {
            yield return null;
        }

        yield return null;
    }

    public override IEnumerator TearDown(TestContext testContext)
    {
        yield return null;
    }

    public override List<IEnumerator> TestCaseList => new()
    {
        TestCameraMovement()
    };

    private IEnumerator TestCameraMovement()
    {
        var cameraBB = Utils.GetBlocksWithType<OVRCameraRig>().First();
        var previousPosition = cameraBB.centerEyeAnchor.position;

        yield return null;

        var newPosition = cameraBB.centerEyeAnchor.position;
        const float epsilon = 0.000001f;
        Assert.Less((newPosition - previousPosition).magnitude, epsilon);

        yield return null;

        var destination = new Vector3(0.0f, 0.0f, 10.0f);
        _sceneManagerTests.SetNodePose(OVRPlugin.Node.EyeCenter, new OVRPlugin.PoseStatef
        {
            Pose = new OVRPlugin.Posef
            {
                Orientation = OVRPlugin.Quatf.identity,
                Position = new OVRPlugin.Vector3f
                {
                    x = destination.x,
                    y = destination.y,
                    z = destination.z
                }
            }
        });

        yield return null;

        newPosition = cameraBB.centerEyeAnchor.position;
        Assert.Greater((newPosition - previousPosition).magnitude, epsilon);
        Assert.Less((newPosition - destination * -1.0f).magnitude, epsilon);
    }
}

#endif //OVRPLUGIN_TESTING
