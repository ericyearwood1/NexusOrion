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

public class RoomModelBuildingBlocksTest : BuildingBlocksTest
{
    public override string BlockId => BlockDataIds.RoomModel;

    public override IEnumerator Setup(TestContext testContext)
    {
        testContext.SceneManagerTestFixture.SceneManager = Utils.GetBlocksWithType<OVRSceneManager>().First();
        testContext.SceneManagerTestFixture.SceneManager.ActiveRoomsOnly = false;
        yield return testContext.SceneManagerTestFixture.LoadRooms(1, false);
    }

    public override IEnumerator TearDown(TestContext testContext)
    {
        testContext.SceneManagerTestFixture.TearDown();
        yield return null;
    }

    public override List<IEnumerator> TestCaseList => new()
    {
        TestShouldLoadRoom()
    };

    private static IEnumerator TestShouldLoadRoom()
    {
        Assert.AreEqual(1, Object.FindObjectsByType<OVRSceneManager>(FindObjectsSortMode.None).Length);
        Assert.AreEqual(1, OVRSceneRoom.SceneRooms.Count);

        using var _ = new OVRObjectPool.ListScope<OVRSceneAnchor>(out var sceneAnchors);
        OVRSceneAnchor.GetSceneAnchors(sceneAnchors);

        Assert.Greater(sceneAnchors.Count, 0);

        yield return null;
    }
}

#endif //OVRPLUGIN_TESTING
