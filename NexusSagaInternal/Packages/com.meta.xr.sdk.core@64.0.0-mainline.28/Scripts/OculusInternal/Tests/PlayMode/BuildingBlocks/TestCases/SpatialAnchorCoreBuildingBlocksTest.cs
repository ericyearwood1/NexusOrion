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
using Meta.XR.BuildingBlocks;
using Meta.XR.BuildingBlocks.Editor;
using NUnit.Framework;
using UnityEngine;

#if OVRPLUGIN_TESTING

public class SpatialAnchorCoreBuildingBlocksTest : BuildingBlocksTest
{
    private SpatialAnchorCoreBuildingBlock _spatialAnchorCore;
    private SceneManagerTests _anchorTestFixture;
    private Guid _anchorGuid = Guid.Empty;

    public override string BlockId => BlockDataIds.SpatialAnchorCore;

    public override IEnumerator Setup(TestContext testContext)
    {
        _anchorTestFixture = testContext.SceneManagerTestFixture;
        yield return new WaitUntil((() => OVRManager.OVRManagerinitialized));
        _spatialAnchorCore = Utils.GetBlocksWithBaseClassType<SpatialAnchorCoreBuildingBlock>().First();
        yield return null;
    }

    public override IEnumerator TearDown(TestContext testContext)
    {
        _spatialAnchorCore.OnAnchorCreateCompleted.RemoveAllListeners();
        _spatialAnchorCore.OnAnchorsLoadCompleted.RemoveAllListeners();
        _spatialAnchorCore.OnAnchorEraseCompleted.RemoveAllListeners();
        _spatialAnchorCore.OnAnchorsEraseAllCompleted.RemoveAllListeners();
        yield return null;
    }

    public override List<IEnumerator> TestCaseList => new()
    {
        TestCreateLoadEraseSpatialAnchor()
    };

    private IEnumerator TestCreateLoadEraseSpatialAnchor()
    {
        yield return TestCreateAnAnchor();
        yield return TestLoadAnAnchor();
        yield return TestEraseAnAnchor();
    }

    private IEnumerator TestCreateAnAnchor()
    {
        OVRSpatialAnchor spatialAnchor = null;
        bool anchorCreated = false;
        _anchorGuid = Guid.Empty;

        _spatialAnchorCore.OnAnchorCreateCompleted.AddListener((anchor, _) =>
        {
            _anchorGuid = anchor.Uuid;
            anchorCreated = true;
        });

        _spatialAnchorCore.InstantiateSpatialAnchor(null, Vector3.zero, Quaternion.identity);
        yield return null;
        _anchorTestFixture.HandleAllSpatialAnchorCreationRequests();

        spatialAnchor = GameObject.FindAnyObjectByType<OVRSpatialAnchor>();
        yield return new WaitUntil(() => spatialAnchor.Created);

        yield return null;

        foreach (var requestId in _anchorTestFixture.SaveSpaceListRequests.Select(pair => pair.Key).ToArray())
        {
            _anchorTestFixture.HandleSaveSpaceListRequest(requestId, OVRPlugin.Result.Success);
        }

        yield return new WaitUntil(() => anchorCreated);

        Assert.True(spatialAnchor.Localized);
        Assert.True(_anchorGuid != Guid.Empty);

        yield return null;
    }

    private IEnumerator TestEraseAnAnchor()
    {
        int eraseCount = 0;
        var eraseAllCompleted = false;
        _spatialAnchorCore.OnAnchorEraseCompleted.AddListener((_, _) => ++eraseCount);
        _spatialAnchorCore.OnAnchorsEraseAllCompleted.AddListener(_ => eraseAllCompleted = true);

        _spatialAnchorCore.EraseAllAnchors();
        yield return null;
        foreach (var (requestId, request) in _anchorTestFixture.EraseQueries.Select(pair => (pair.Key, pair.Value)).ToArray())
        {
            _anchorTestFixture.HandleEraseQuery(requestId, OVRPlugin.Result.Success, _anchorGuid, OVRPlugin.SpaceStorageLocation.Local);
        }

        yield return new WaitUntil(() => eraseAllCompleted);

        Assert.Greater(eraseCount, 0);
        Assert.IsEmpty(GameObject.FindObjectsByType<OVRSpatialAnchor>(FindObjectsSortMode.None));

        yield return null;
    }

    private IEnumerator TestLoadAnAnchor()
    {
        bool loadCompleted = false;
        _spatialAnchorCore.OnAnchorsLoadCompleted.AddListener(_ => loadCompleted = true);

        var spatialAnchors = GameObject.FindObjectsByType<OVRSpatialAnchor>(FindObjectsSortMode.None);
        for (int i = 0; i < spatialAnchors.Length; i++)
        {
            GameObject.Destroy(spatialAnchors[i].gameObject);
        }
        yield return null;

        var space = new AnchorTestFixture.SpatialAnchor(_anchorGuid);
        _anchorTestFixture.AddSpace(space);

        var prefab = new GameObject("OVRSpatialAnchor");
        _spatialAnchorCore.LoadAndInstantiateAnchors(prefab, new List<Guid>() { _anchorGuid });
        _anchorTestFixture.HandleAllSpaceQueries();
        yield return null;

        foreach (var (requestId, request) in _anchorTestFixture.ComponentStatusChangeRequests.Select(Extensions.Deconstruct).ToArray())
        {
            if (request.Space.Handle == space.Handle)
            {
                var response = new OVRDeserialize.SpaceSetComponentStatusCompleteData
                {
                    RequestId = requestId,
                    Result = (int)OVRPlugin.Result.Success,
                    Space = space,
                    Uuid = space.Uuid,
                    ComponentType = request.ComponentType,
                    Enabled = (int)OVRPlugin.Bool.True
                };
                _anchorTestFixture.HandleComponentStatusChangeRequest(response);
            }
        }

        yield return null;

        Assert.True(loadCompleted);
        var spatialAnchor = GameObject.FindAnyObjectByType<OVRSpatialAnchor>();
        Assert.IsNotNull(spatialAnchor);
        Assert.True(spatialAnchor.Uuid == _anchorGuid);

        yield return null;
    }
}

#endif //OVRPLUGIN_TESTING
