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

public class PassthroughWindowBuildingBlocksTest : BuildingBlocksTest
{
    public override string BlockId => BlockDataIds.PassthroughWindow;

    public override IEnumerator Setup(TestContext testContext)
    {
        yield return new WaitUntil(() => OVRManager.OVRManagerinitialized);
        yield return null;
    }

    public override IEnumerator TearDown(TestContext testContext)
    {
        yield return null;
    }

    public override List<IEnumerator> TestCaseList => new()
    {
        TestInstalledProperly(),
        TestInstallOnGameObject()
    };

    private IEnumerator TestInstalledProperly()
    {
        var passthroughLayers = GameObject.FindObjectsByType<OVRPassthroughLayer>(FindObjectsSortMode.None);
        Assert.True(passthroughLayers.Any());
        Assert.True(Object.FindObjectsByType<Transform>(FindObjectsSortMode.None).Any(t => t.name == "Quad"));
        yield return null;
    }

    private IEnumerator TestInstallOnGameObject()
    {
        var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        var ptWindowBlock = Utils.PublicBlockData.OfType<PassthroughWindowBlockData>().First();
        ptWindowBlock.InstallWithDependencies(cube);
        yield return null;

        var ptMaterial = ptWindowBlock.Prefab.GetComponentInChildren<MeshRenderer>().sharedMaterial;
        Assert.True(cube.GetComponent<MeshRenderer>().sharedMaterial == ptMaterial);

        yield return null;
    }
}

#endif //OVRPLUGIN_TESTING
