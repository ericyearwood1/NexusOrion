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
using NUnit.Framework;
using UnityEngine;
using Meta.XR.BuildingBlocks.Editor;

#if OVRPLUGIN_TESTING

public class SampleSpatialAnchorControllerBuildingBlocksTest : BuildingBlocksTest
{
    public override string BlockId => BlockDataIds.SampleSpatialAnchorController;
    private SpatialAnchorCoreBuildingBlock _spatialAnchorCore;
    private AnchorTestFixture _testFixture;
    private Guid _anchorGuid;
    private ControllerButtonsMapper _controllerButtonMapper;

    public override IEnumerator Setup(TestContext testContext)
    {
        _testFixture = testContext.SceneManagerTestFixture;
        _spatialAnchorCore = Utils.GetBlocksWithBaseClassType<SpatialAnchorCoreBuildingBlock>().First();
        _controllerButtonMapper = Utils.GetBlocksWithType<ControllerButtonsMapper>().First();

        yield return new WaitUntil((() => OVRManager.OVRManagerinitialized));
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
        RunTestsInSequence()
    };

    private IEnumerator RunTestsInSequence()
    {
        yield return TestShouldCreateAnchorOnAssignedButtonPress();
        yield return TestShouldLoadAnchorOnAssignedButtonPress();
        yield return TestShouldEraseAnchorOnAssignedButtonPress();
    }

    private IEnumerator TestShouldCreateAnchorOnAssignedButtonPress()
    {
        OVRSpatialAnchor spatialAnchor = null;
        bool anchorCreated = false;
        _anchorGuid = Guid.Empty;

        _spatialAnchorCore.OnAnchorCreateCompleted.AddListener((anchor, _) =>
        {
            _anchorGuid = anchor.Uuid;
            anchorCreated = true;
        });

        var createButton = _controllerButtonMapper.ButtonClickActions.First(action => action.Title.Equals("Spawn spatial anchor")).Button;
        yield return OnButtonUp(createButton);

        _testFixture.HandleAllSpatialAnchorCreationRequests();
        spatialAnchor = GameObject.FindAnyObjectByType<OVRSpatialAnchor>();
        yield return new WaitUntil(() => spatialAnchor.Created);
        yield return null;
        foreach (var requestId in _testFixture.SaveSpaceListRequests.Select(pair => pair.Key).ToArray())
        {
            _testFixture.HandleSaveSpaceListRequest(requestId, OVRPlugin.Result.Success);
        }
        yield return new WaitUntil(() => anchorCreated);

        Assert.True(spatialAnchor.Localized);
        Assert.True(_anchorGuid != Guid.Empty);

        yield return null;
    }

    private IEnumerator TestShouldLoadAnchorOnAssignedButtonPress()
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
        _testFixture.AddSpace(space);

        var loadButton = _controllerButtonMapper.ButtonClickActions.First(action => action.Title.Equals("Load and spawn spatial anchors")).Button;
        yield return OnButtonUp(loadButton);

        yield return HandleSpaceQueries(space);

        Assert.True(loadCompleted);
        var spatialAnchor = GameObject.FindAnyObjectByType<OVRSpatialAnchor>();
        Assert.True(spatialAnchor.Uuid == _anchorGuid);

        yield return null;
    }

    private IEnumerator TestShouldEraseAnchorOnAssignedButtonPress()
    {
        var eraseAllCompleted = false;
        _spatialAnchorCore.OnAnchorsEraseAllCompleted.AddListener(_ => eraseAllCompleted = true);

        var eraseButton = _controllerButtonMapper.ButtonClickActions.First(action => action.Title.Equals("Erase all spatial anchors")).Button;
        yield return OnButtonUp(eraseButton);

        foreach (var (requestId, request) in _testFixture.EraseQueries.Select(pair => (pair.Key, pair.Value)).ToArray())
        {
            _testFixture.HandleEraseQuery(requestId, OVRPlugin.Result.Success, _anchorGuid, OVRPlugin.SpaceStorageLocation.Local);
        }

        yield return new WaitUntil(() => eraseAllCompleted);
        Assert.IsEmpty(GameObject.FindObjectsByType<OVRSpatialAnchor>(FindObjectsSortMode.None));

        yield return null;
    }

    private IEnumerator HandleSpaceQueries(AnchorTestFixture.SpatialAnchor space)
    {
        _testFixture.HandleAllSpaceQueries();
        yield return null;

        foreach (var (requestId, request) in _testFixture.ComponentStatusChangeRequests.Select(Extensions.Deconstruct).ToArray())
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
                _testFixture.HandleComponentStatusChangeRequest(response);
            }
        }
        yield return null;
    }

    private IEnumerator OnButtonUp(OVRInput.Button btn)
    {
        _testFixture.SetOVRInputState(btn);
        yield return null;

        _testFixture.SetOVRInputState(OVRInput.Button.None);
        yield return null;
    }
}

#endif //OVRPLUGIN_TESTING
