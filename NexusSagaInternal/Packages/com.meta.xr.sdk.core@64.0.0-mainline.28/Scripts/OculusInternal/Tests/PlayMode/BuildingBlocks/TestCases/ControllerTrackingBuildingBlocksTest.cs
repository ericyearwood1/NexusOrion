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

public class ControllerTrackingBuildingBlocksTest : BuildingBlocksTest
{
    public override string BlockId => BlockDataIds.ControllerTracking;
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
        TestControllerMovement()
    };

    private IEnumerator TestControllerMovement()
    {
        var leftControllerBuildingBlock = Utils.GetBlocksWithType<OVRControllerHelper>()
            .First(c => c.m_controller == OVRInput.Controller.LTouch);
        var rightControllerBuildingBlock = Utils.GetBlocksWithType<OVRControllerHelper>()
            .First(c => c.m_controller == OVRInput.Controller.RTouch);

        var previousLeftPosition = leftControllerBuildingBlock.transform.position;
        var previousRightPosition = rightControllerBuildingBlock.transform.position;

        yield return null;

        const float epsilon = 0.000001f;
        Assert.Less((leftControllerBuildingBlock.transform.position - previousLeftPosition).magnitude, epsilon);
        Assert.Less((rightControllerBuildingBlock.transform.position - previousRightPosition).magnitude, epsilon);

        yield return null;

        var destinationLeft = new Vector3(0.0f, 0.0f, 2.0f);
        var destinationRight = new Vector3(0.0f, 0.0f, -2.0f);
        SetNodePose(OVRPlugin.Node.ControllerLeft, destinationLeft);
        SetNodePose(OVRPlugin.Node.ControllerRight, destinationRight);

        yield return null;

        Assert.Greater((leftControllerBuildingBlock.transform.position - previousLeftPosition).magnitude, epsilon);
        Assert.Less((leftControllerBuildingBlock.transform.position - destinationLeft * -1.0f).magnitude, epsilon);

        Assert.Greater((rightControllerBuildingBlock.transform.position - previousRightPosition).magnitude, epsilon);
        Assert.Less((rightControllerBuildingBlock.transform.position - destinationRight * -1.0f).magnitude, epsilon);
    }

    private void SetNodePose(OVRPlugin.Node nodeId, Vector3 destination)
    {
        _sceneManagerTests.SetNodePose(nodeId, new OVRPlugin.PoseStatef
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
    }
}

#endif //OVRPLUGIN_TESTING
