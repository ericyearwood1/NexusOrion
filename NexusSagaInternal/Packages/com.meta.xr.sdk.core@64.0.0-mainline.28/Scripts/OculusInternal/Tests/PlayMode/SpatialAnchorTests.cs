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

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System.Threading.Tasks;
using Object = UnityEngine.Object;
using Random = UnityEngine.Random;

internal class SpatialAnchorTests : AnchorTestFixture
{
    static float[] _scales = { -1, .5f, 1, 2, 10, 100 };

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator AnchorPositionIsInvariantUnderDifferentCameraRigScales(
        [ValueSource(nameof(_scales))] float scale)
    {
        CameraRig.transform.localScale = Vector3.one * scale;
        CameraRig.transform.position = Random.insideUnitSphere * 100f;
        CameraRig.transform.rotation = Random.rotationUniform;

        var anchor = new GameObject("anchor").AddComponent<OVRSpatialAnchor>();
        var anchorPosition = Random.insideUnitSphere * 100f;
        anchor.transform.position = anchorPosition;

        yield return null;

        Assert.AreEqual(1, SpatialAnchorCreationRequests.Count);

        var createInfo = default(OVRPlugin.SpatialAnchorCreateInfo);
        foreach (var (requestId, request) in SpatialAnchorCreationRequests
                     .Select(kvp => (kvp.Key, kvp.Value)).ToArray())
        {
            var space = new SpatialAnchor();
            createInfo = request;
            HandleSpatialAnchorCreationRequest(new OVRDeserialize.SpatialAnchorCreateCompleteData
            {
                RequestId = requestId,
                Result = (int)OVRPlugin.Result.Success,
                Space = space,
                Uuid = space.Uuid
            }, space);
        }

        yield return new WaitWhile(() => anchor.PendingCreation);

        Assert.True(anchor.Created);
        Assert.False(anchor.Localized);

        yield return new WaitWhile(() => ComponentStatusChangeRequests.Count == 0);

        foreach (var (requestId, request) in ComponentStatusChangeRequests
                     .Select(kvp => (kvp.Key, kvp.Value)).ToArray())
        {
            if (request.ComponentType != OVRPlugin.SpaceComponentType.Locatable) continue;

            Assert.True(Spaces.TryGetValue(request.Space, out var space));
            Assert.True(space is SpatialAnchor);

            var spatialAnchor = (SpatialAnchor)space;

            var response = new OVRDeserialize.SpaceSetComponentStatusCompleteData
            {
                ComponentType = request.ComponentType,
                Enabled = (int)OVRPlugin.Bool.False,
                RequestId = requestId,
                Result = (int)OVRPlugin.Result.Failure_Unsupported,
                Space = spatialAnchor,
                Uuid = spatialAnchor.Uuid
            };

            spatialAnchor.AddComponent<Locatable>().SpaceLocation.pose = createInfo.PoseInSpace;
            response.Enabled = (int)OVRPlugin.Bool.True;
            response.Result = (int)OVRPlugin.Result.Success;

            HandleComponentStatusChangeRequest(response);
        }

        yield return null;

        // The ==operator is a bit too conservative and uses (approx) sqrMagnitud < 1e-11
        Assert.Less((anchorPosition - anchor.transform.position).sqrMagnitude, 1e-6,
            $"Anchor position {anchor.transform.position.ToString("f6")} did not match expected {anchorPosition.ToString("f6")}");

        Object.DestroyImmediate(anchor);
    }

    public async Task<OVRSpatialAnchor> CreateAnchor(OVRPlugin.Result ExpectedCreationResult,
        OVRPlugin.Result ExpectedLocalizeResult, bool isLocalizationQuerySuccess)
    {
        SetComponentStatusSucceed = isLocalizationQuerySuccess;
        var anchor = new GameObject($"anchor").AddComponent<OVRSpatialAnchor>();

        var localizationResults = new Dictionary<OVRSpatialAnchor.OperationResult, int>();

        anchor.OnLocalize += OnLocalizationComplete;

        void OnLocalizationComplete(OVRSpatialAnchor.OperationResult result)
        {
            localizationResults[result] = GetLocalizationResults(localizationResults, result) + 1;
        }

        // Due to the test framework, the test needs to wait 2 frames to have
        // the Start called before the rest of the test.
        await Task.Yield();
        await Task.Yield();

        foreach (var (requestId, request) in SpatialAnchorCreationRequests
                     .Select(kvp => (kvp.Key, kvp.Value)).ToArray())
        {
            var space = new SpatialAnchor();
            HandleSpatialAnchorCreationRequest(new OVRDeserialize.SpatialAnchorCreateCompleteData
            {
                RequestId = requestId,
                Result = (int)ExpectedCreationResult,
                Space = space,
                Uuid = space.Uuid
            }, space);
        }

        await Task.Yield();

        if (ExpectedCreationResult == OVRPlugin.Result.Success)
        {
            await Task.Run(async () =>
            {
                while (anchor.PendingCreation) await Task.Yield();
            });

            await CheckLocalizeAnchorInternal(localizationResults, anchor, ExpectedLocalizeResult,
                isLocalizationQuerySuccess);
        }
        else
        {
            Assert.IsFalse(anchor, "the anchor should not exist");
        }

        anchor.OnLocalize -= OnLocalizationComplete;

        return anchor;
    }

    public async Task<OVRSpatialAnchor> CreateAnchorUsingInitializeFromExisting(Guid anchorUuid, ulong anchorSpace,
        OVRPlugin.Result ExpectedLocalizeResult, bool isLocalizationQuerySuccess)
    {
        SetComponentStatusSucceed = isLocalizationQuerySuccess;
        var anchor = new GameObject($"anchor").AddComponent<OVRSpatialAnchor>();
#pragma warning disable CS0618
        anchor.InitializeFromExisting(anchorSpace, anchorUuid);
#pragma warning restore

        var localizationResults = new Dictionary<OVRSpatialAnchor.OperationResult, int>();

        anchor.OnLocalize += OnLocalizationComplete;

        void OnLocalizationComplete(OVRSpatialAnchor.OperationResult result)
        {
            localizationResults[result] = GetLocalizationResults(localizationResults, result) + 1;
        }

        // Need to wait 2 frames to the Creation call to occur
        await Task.Yield();
        await Task.Yield();

        await CheckLocalizeAnchorInternal(localizationResults, anchor, ExpectedLocalizeResult,
            isLocalizationQuerySuccess);

        anchor.OnLocalize -= OnLocalizationComplete;

        return anchor;
    }


    private int GetLocalizationResults(Dictionary<OVRSpatialAnchor.OperationResult, int> localizationResults,
        OVRSpatialAnchor.OperationResult result)
    {
        localizationResults.TryGetValue(result, out var counter);
        return counter;
    }

    private async Task CheckLocalizeAnchorInternal(
        Dictionary<OVRSpatialAnchor.OperationResult, int> localizationResults, OVRSpatialAnchor anchor,
        OVRPlugin.Result ExpectedLocalizeResult, bool isLocalizationQuerySuccess)
    {
        Assert.IsTrue(anchor.Created, "anchor should be created");
        Assert.False(anchor.Localized, "anchor should not be localized");
        Assert.AreEqual(0, GetLocalizationResults(localizationResults, OVRSpatialAnchor.OperationResult.Success),
            "The number of localization callback should be 0");


        if (isLocalizationQuerySuccess)
        {
            // Check Status Change: Locatable, Sharable, Storable
            Assert.AreEqual(3, ComponentStatusChangeRequests.Count,
                "the number of component status change requests should be 3");
            Assert.AreEqual(1, ComponentStatusChangeRequests.Values
                    .Count(r => r.ComponentType == OVRPlugin.SpaceComponentType.Locatable),
                "the number of locatable status change requests should be 1");

            foreach (var (requestId, request) in ComponentStatusChangeRequests
                         .Select(kvp => (kvp.Key, kvp.Value)).ToArray())
            {
                Assert.True(Spaces.TryGetValue(request.Space, out var space), "the space should be found");
                Assert.True(space is SpatialAnchor, "the space should be of type SpatialAnchor");

                var spatialAnchor = (SpatialAnchor)space;

                var response = new OVRDeserialize.SpaceSetComponentStatusCompleteData
                {
                    ComponentType = request.ComponentType,
                    Enabled = ExpectedLocalizeResult == OVRPlugin.Result.Success
                        ? (int)OVRPlugin.Bool.True
                        : (int)OVRPlugin.Bool.False,
                    RequestId = requestId,
                    Result = (int)ExpectedLocalizeResult,
                    Space = spatialAnchor,
                    Uuid = spatialAnchor.Uuid
                };

                HandleComponentStatusChangeRequest(response);
            }

            await Task.Yield();

            Assert.AreNotEqual(Guid.Empty, anchor.Uuid,
                "The Uuid should be properly initialized and not equal to empty Guid.");
            Assert.True(anchor.Created, "Space of the anchor should be valid");
            if (ExpectedLocalizeResult == OVRPlugin.Result.Success)
            {
                Assert.IsTrue(anchor.Localized, "anchor should be localized");
                Assert.AreEqual(1,
                    GetLocalizationResults(localizationResults, OVRSpatialAnchor.OperationResult.Success),
                    "the number of localization callback for success should be 1");
            }
            else
            {
                Assert.IsFalse(anchor.Localized, "anchor should not be localized");
                Assert.AreEqual(1,
                    GetLocalizationResults(localizationResults, OVRSpatialAnchor.OperationResult.Failure),
                    "the number of localization callback for failure should be 1");
            }
        }
        else
        {
            await Task.Yield();
            Assert.AreEqual(0, ComponentStatusChangeRequests.Count, "the number of status change request should be 0");
            Assert.False(anchor.Localized, "anchor should not be localized");
            Assert.True(anchor.Created, "the anchor space should be valid");
        }
    }

    public async Task<bool> EraseAnchors(OVRSpatialAnchor anchorToErase, OVRPlugin.SpaceStorageLocation storageLocation,
        OVRPlugin.Result ExpectedEraseResult, bool isEraseQuerySuccess)
    {
        bool onEraseResult = false;

        void OnErase(OVRSpatialAnchor anchor, bool success)
        {
            onEraseResult = success;
        }

        Assert.AreEqual(0, EraseQueries.Count, "Number of Erase query should be 0");
#pragma warning disable CS0618
        anchorToErase.Erase(OnErase);
#pragma warning restore

        await Task.Yield();

        // call handle only if the query succeed
        if (isEraseQuerySuccess)
        {
            Assert.AreEqual(1, EraseQueries.Count, "Only one Erase query should have been made.");
            var (requestId, space) = EraseQueries.Select(pair => (pair.Key, pair.Value)).ToArray()[0];

#pragma warning disable CS0618
            Assert.IsTrue(anchorToErase.Space == space,
                "the space from the query should be the same than the space of the anchor to erase");
            anchorToErase.Space.TryGetUuid(out var uuid);
#pragma warning restore
            HandleEraseQuery(requestId, ExpectedEraseResult, uuid, storageLocation);
        }

        await Task.Yield();

        Assert.AreEqual(0, EraseQueries.Count, "the number of Erase query should be 0");

        return onEraseResult;
    }

    public async Task<List<OVRSpatialAnchor.UnboundAnchor>> LoadAnchors(List<Guid> ExistingUuids, List<Guid> loadUuids,
        bool isSpaceQuerySuccess)
    {
        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult = new List<OVRSpatialAnchor.UnboundAnchor>();

        foreach (var uuid in ExistingUuids)
        {
            var space = new SpatialAnchor(uuid);
            AddSpace(space);
        }

        void OnLoadComplete(OVRSpatialAnchor.UnboundAnchor[] unboundsAnchors)
        {
            unboundsAnchorsResult =
                new List<OVRSpatialAnchor.UnboundAnchor>(unboundsAnchors ??
                                                         Array.Empty<OVRSpatialAnchor.UnboundAnchor>());
        }

#pragma warning disable CS0618
        bool result = OVRSpatialAnchor.LoadUnboundAnchors(new OVRSpatialAnchor.LoadOptions
        {
            StorageLocation = OVRSpace.StorageLocation.Local,
            Timeout = 0,
            Uuids = loadUuids,
        }, OnLoadComplete);
#pragma warning restore

        await Task.Yield();
        if (isSpaceQuerySuccess)
        {
            Assert.IsTrue(result, "the result of the call LoadUnboundAnchors should be true (Succeed)");
            Assert.AreEqual(1, SpaceQueries.Count, "the number of SpaceQuery should be 1.");

            HandleAllSpaceQueries();
        }
        else
        {
            Assert.IsFalse(result, "the result of the call LoadUnboundAnchors should be false (Failure)");
            Assert.AreEqual(0, SpaceQueries.Count, "the number of SpaceQuery should be 0.");
        }

        await Task.Yield();

        return unboundsAnchorsResult;
    }

    public async Task<List<OVRSpatialAnchor.UnboundAnchor>> LoadAnchorsAsync(List<Guid> ExistingUuids,
        List<Guid> loadUuids, bool isSpaceQuerySuccess)
    {
        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult = new List<OVRSpatialAnchor.UnboundAnchor>();

        foreach (var uuid in ExistingUuids)
        {
            var space = new SpatialAnchor(uuid);
            AddSpace(space);
        }

        var task = OVRSpatialAnchor.LoadUnboundAnchorsAsync(new OVRSpatialAnchor.LoadOptions
        {
            StorageLocation = OVRSpace.StorageLocation.Local,
            Timeout = 0,
            Uuids = loadUuids,
        });

        await Task.Yield();

        if (isSpaceQuerySuccess)
        {
            Assert.IsFalse(task.IsCompleted);
            Assert.AreEqual(1, SpaceQueries.Count, "the number of SpaceQuery should be 1.");

            HandleAllSpaceQueries();
        }
        else
        {
            Assert.IsTrue(task.IsCompleted);
            Assert.AreEqual(0, SpaceQueries.Count, "the number of SpaceQuery should be 0.");
        }

        await Task.Yield();

        return (task.GetResult() ?? Array.Empty<OVRSpatialAnchor.UnboundAnchor>()).ToList();
    }

    public async Task LocalizeUnboundAnchors(List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult,
        OVRPlugin.Result expectedLocalizeResult, int expectedLocalizedAnchorsCount)
    {
        int localizedUnboundAnchorCount = 0;

        void OnLocalizeComplete(OVRSpatialAnchor.UnboundAnchor unboundsAnchor, bool result)
        {
            if (expectedLocalizeResult == OVRPlugin.Result.Success)
            {
                Assert.IsTrue(unboundsAnchor.Localized, "unboundAnchor is expected to be localized.");
                Assert.IsTrue(result, "result of the localize callback should be successful.");
                localizedUnboundAnchorCount++;
            }
            else
            {
                Assert.IsFalse(unboundsAnchor.Localized, "unboundAnchor is expected to not be localized.");
                Assert.IsFalse(result, "result of the localize callback should be failing.");
            }
        }

        foreach (var unboundAnchor in unboundsAnchorsResult)
        {
#pragma warning disable CS0618
            unboundAnchor.Localize(OnLocalizeComplete);
#pragma warning restore

            await Task.Yield();

            foreach (var (requestId, request) in ComponentStatusChangeRequests.Select(Extensions.Deconstruct).ToArray())
            {
                if (request.Space.Handle == unboundAnchor._space.Handle)
                {
                    var response = new OVRDeserialize.SpaceSetComponentStatusCompleteData
                    {
                        RequestId = requestId,
                        Result = (int)expectedLocalizeResult,
                        Space = unboundAnchor._space,
                        Uuid = unboundAnchor.Uuid,
                        ComponentType = request.ComponentType,
                        Enabled = expectedLocalizeResult == OVRPlugin.Result.Success
                            ? (int)OVRPlugin.Bool.True
                            : (int)OVRPlugin.Bool.False
                    };
                    HandleComponentStatusChangeRequest(response);
                }
            }
        }

        await Task.Yield();

        Assert.AreEqual(expectedLocalizedAnchorsCount, localizedUnboundAnchorCount,
            "the number of anchor to be localized (pass the callback)");
    }

    public async Task SaveAnchor(OVRSpatialAnchor anchor, OVRSpace.StorageLocation storageLocation,
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor, OVRPlugin.Result result, bool saveQuerySuccess)
    {
        SaveSpaceListSucceed = saveQuerySuccess;

        void OnSave(OVRSpatialAnchor anchor, bool success)
        {
            savedResultByAnchor[anchor] = success;
        }

#pragma warning disable CS0618
        anchor.Save(OnSave);
#pragma warning restore

        await Task.Yield();

        if (saveQuerySuccess)
        {
            Assert.AreEqual(1, SaveSpaceListRequests.Count, "the number of SaveQuery should be 1.");

            foreach (var requestId in SaveSpaceListRequests.Select(pair => pair.Key).ToArray())
            {
                HandleSaveSpaceListRequest(requestId, result);
            }

            await Task.Yield();
        }
        else
        {
            Assert.AreEqual(0, SaveSpaceListRequests.Count, "the number of SaveQuery should be 0.");
        }
    }

    public async Task SaveAnchorAsync(OVRSpatialAnchor anchor, OVRSpace.StorageLocation storageLocation,
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor, OVRPlugin.Result result, bool saveQuerySuccess)
    {
        SaveSpaceListSucceed = saveQuerySuccess;

        var task = anchor.SaveAsync(new OVRSpatialAnchor.SaveOptions
        {
            Storage = OVRSpace.StorageLocation.Local
        });

        Assert.IsFalse(task.IsCompleted);
        await Task.Yield();

        if (saveQuerySuccess)
        {
            Assert.AreEqual(1, SaveSpaceListRequests.Count, "the number of SaveQuery should be 1.");

            foreach (var requestId in SaveSpaceListRequests.Select(pair => pair.Key).ToArray())
            {
                HandleSaveSpaceListRequest(requestId, result);
            }

            await Task.Yield();
        }
        else
        {
            Assert.AreEqual(0, SaveSpaceListRequests.Count, "the number of SaveQuery should be 0.");
        }

        await Task.Yield();

        Assert.IsTrue(task.IsCompleted);
        Assert.AreEqual(saveQuerySuccess, task.GetResult());
        savedResultByAnchor[anchor] = task.GetResult();
    }

    public async Task<Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>> SaveAnchorStatic(
        List<OVRSpatialAnchor> anchors, OVRSpace.StorageLocation storageLocation, OVRPlugin.Result result,
        bool saveQuerySuccess, bool useLegacy)
    {
        SaveSpaceListSucceed = saveQuerySuccess;
        var shareResults = new Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>();

        void OnSave(ICollection<OVRSpatialAnchor> savedAnchors, OVRSpatialAnchor.OperationResult success)
        {
            foreach (var anchor in savedAnchors)
            {
                shareResults[anchor] = success;
            }
        }

        if (useLegacy)
        {
#pragma warning disable CS0618
            OVRSpatialAnchor.Save(anchors, new OVRSpatialAnchor.SaveOptions { Storage = storageLocation }, OnSave);
#pragma warning restore
        }
        else
        {
            OVRSpatialAnchor.SaveAsync(anchors, new OVRSpatialAnchor.SaveOptions { Storage = storageLocation })
                .ContinueWith((taskResult, savedAnchors) => OnSave(savedAnchors, taskResult), anchors);
        }

        await Task.Yield();

        if (saveQuerySuccess)
        {
            Assert.AreEqual(1, SaveSpaceListRequests.Count, "the number of save query should be equal to 1.");

            foreach (var requestId in SaveSpaceListRequests.Select(pair => pair.Key).ToArray())
            {
                HandleSaveSpaceListRequest(requestId, result);
            }

            await Task.Yield();
        }
        else
        {
            Assert.AreEqual(0, SaveSpaceListRequests.Count, "the number of save query should all be processed");
        }

        return shareResults;
    }


    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanSaveAsync([Values] bool saveResult)
    {
        var anchor = new GameObject("anchor").AddComponent<OVRSpatialAnchor>();
        yield return null;

        foreach (var (requestId, request) in SpatialAnchorCreationRequests
                     .Select(kvp => (kvp.Key, kvp.Value)).ToArray())
        {
            var space = new SpatialAnchor();
            HandleSpatialAnchorCreationRequest(new OVRDeserialize.SpatialAnchorCreateCompleteData
            {
                RequestId = requestId,
                Result = (int)OVRPlugin.Result.Success,
                Space = space,
                Uuid = space.Uuid
            }, space);
        }

        yield return new WaitWhile(() => anchor.PendingCreation);

        var task = anchor.SaveAsync();
        Assert.IsFalse(task.IsCompleted);
        yield return null;

        Assert.AreEqual(1, SaveSpaceListRequests.Count);

        var loggerState = Debug.unityLogger.logEnabled;
        if (!saveResult)
        {
            // disabling the logs to avoid logging error logs during the tests that remain in the console
            Debug.unityLogger.logEnabled = false;
        }

        foreach (var requestId in SaveSpaceListRequests.Select(pair => pair.Key).ToArray())
        {
            HandleSaveSpaceListRequest(requestId, saveResult ? OVRPlugin.Result.Success : OVRPlugin.Result.Failure);
        }

        yield return null;

        if (!saveResult)
        {
            Debug.unityLogger.logEnabled = loggerState;
        }

        Assert.IsTrue(task.IsCompleted);
        Assert.AreEqual(saveResult, task.GetResult());

        yield return null;
        Object.DestroyImmediate(anchor);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanShareAsync([Values] bool shareResult)
    {
        var anchor = new GameObject("anchor").AddComponent<OVRSpatialAnchor>();
        yield return null;

        foreach (var (requestId, request) in SpatialAnchorCreationRequests
                     .Select(kvp => (kvp.Key, kvp.Value)).ToArray())
        {
            var space = new SpatialAnchor();
            HandleSpatialAnchorCreationRequest(new OVRDeserialize.SpatialAnchorCreateCompleteData
            {
                RequestId = requestId,
                Result = (int)OVRPlugin.Result.Success,
                Space = space,
                Uuid = space.Uuid
            }, space);
        }

        yield return new WaitWhile(() => anchor.PendingCreation);

        var user = new OVRSpaceUser(123);
        var task = anchor.ShareAsync(user);
        Assert.IsFalse(task.IsCompleted);
        yield return null;

        Assert.AreEqual(1, ShareQueries.Count);

        var loggerState = Debug.unityLogger.logEnabled;
        if (!shareResult)
        {
            // disabling the logs to avoid logging error logs during the tests that remain in the console
            Debug.unityLogger.logEnabled = false;
        }

        foreach (var requestId in ShareQueries.Select(pair => pair.Key).ToArray())
        {
            HandleShareQuery(requestId, shareResult ? OVRPlugin.Result.Success : OVRPlugin.Result.Failure);
        }

        yield return null;

        if (!shareResult)
        {
            Debug.unityLogger.logEnabled = loggerState;
        }

        Assert.IsTrue(task.IsCompleted);
        var expectedResult =
            shareResult ? OVRSpatialAnchor.OperationResult.Success : OVRSpatialAnchor.OperationResult.Failure;
        Assert.AreEqual(expectedResult, task.GetResult());
        Object.DestroyImmediate(anchor);
    }


    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanLocalizeAsync([Values] bool localizeResult)
    {
        var space = new SpatialAnchor();
        AddSpace(space);
        var uuid = Guid.NewGuid();
        var unboundAnchor = new OVRSpatialAnchor.UnboundAnchor(space.Handle, uuid);

        var task = unboundAnchor.LocalizeAsync();
        Assert.IsFalse(task.IsCompleted);
        yield return null;

        Assert.IsTrue(ComponentStatusChangeRequests.Values.Select(req => req.Space)
            .All(reqSpace => reqSpace == space));

        var componentSet = new HashSet<OVRPlugin.SpaceComponentType>
        {
            OVRPlugin.SpaceComponentType.Locatable, OVRPlugin.SpaceComponentType.Storable,
            OVRPlugin.SpaceComponentType.Sharable
        };
        var reqComponentSet = new HashSet<OVRPlugin.SpaceComponentType>
            (ComponentStatusChangeRequests.Values.Select(req => req.ComponentType));
        Assert.IsTrue(reqComponentSet.SetEquals(componentSet));

        yield return null;

        foreach (var (requestId, data) in ComponentStatusChangeRequests.Select(Extensions.Deconstruct).ToArray())
        {
            HandleComponentStatusChangeRequest(new OVRDeserialize.SpaceSetComponentStatusCompleteData
            {
                RequestId = requestId,
                Result = localizeResult ? 0 : -1,
                Space = data.Space,
                Uuid = uuid,
                ComponentType = data.ComponentType,
                Enabled = data.Enable == OVRPlugin.Bool.True ? 1 : 0
            });
        }

        yield return null;

        Assert.IsTrue(task.IsCompleted);
        Assert.AreEqual(localizeResult, task.GetResult());
    }

    private struct ShareRequest
    {
        public ulong AnchorSpace;
        public List<ulong> UserHandles;
    };

    public async Task ShareAnchorsUsingList(List<OVRSpatialAnchor> anchors, List<OVRSpaceUser[]> userLists,
        Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult> shareResults,
        OVRPlugin.Result expectedShareResult, int numberOfQueries, bool shareQueryResult)
    {
        var shareRequests = new List<ShareRequest>();
        for (var i = 0; i < anchors.Count; i++)
        {
            var anchor = anchors[i];

            void OnShare(OVRSpatialAnchor.OperationResult result)
            {
                shareResults[anchor] = result;
            }

#pragma warning disable CS0618
            anchor.Share(userLists[i], OnShare);

            shareRequests.Add(new ShareRequest
            {
                AnchorSpace = anchor.Space.Handle,
                UserHandles = userLists[i].Select(user => user._handle).ToList()
            });
#pragma warning restore
        }

        await Task.Yield();
        if (shareQueryResult)
        {
            ShareAnchorsHandle(shareRequests, expectedShareResult, numberOfQueries);
        }

        Assert.AreEqual(0, ShareQueries.Count, "all share query is supposed to be handle.");

        await Task.Yield();

        Assert.AreEqual(anchors.Count, shareResults.Count, "the number of results should equal the number of anchors");
    }

    private void ShareAnchorsHandle(List<ShareRequest> shareRequests, OVRPlugin.Result expectedShareResult,
        int numberOfQueries)
    {
        Assert.AreEqual(numberOfQueries, ShareQueries.Count);

        foreach (var pair in ShareQueries)
        {
            var queryData = pair.Value;

            var queryUsers = new HashSet<ulong>(queryData.UserHandles.Select(handle => SpaceUsers[handle]));

            foreach (var request in shareRequests.Where(request => queryData.Spaces.Contains(request.AnchorSpace)))
            {
                var requestUsers = new HashSet<ulong>(request.UserHandles.Select(handle => SpaceUsers[handle]));
                Assert.IsTrue(queryUsers.SetEquals(requestUsers),
                    "user from the sharerequest input should appear in the the register ShareQueries user");
            }
        }

        foreach (var requestId in ShareQueries.Select(pair => pair.Key).ToArray())
        {
            HandleShareQuery(requestId, expectedShareResult);
        }
    }

    public async Task ShareAnchors(List<OVRSpatialAnchor> anchors, List<OVRSpaceUser> userLists,
        Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult> shareResults,
        OVRPlugin.Result expectedShareResult, int numberOfQueries)
    {
        var shareRequests = new List<ShareRequest>();
        for (var i = 0; i < anchors.Count; i++)
        {
            var anchor = anchors[i];

            void OnShare(OVRSpatialAnchor.OperationResult result)
            {
                shareResults[anchor] = result;
            }

#pragma warning disable CS0618
            switch (userLists.Count)
            {
                case 1:
                    anchor.Share(userLists[0], OnShare);
                    break;
                case 2:
                    anchor.Share(userLists[0], userLists[1], OnShare);
                    break;
                case 3:
                    anchor.Share(userLists[0], userLists[1], userLists[2], OnShare);
                    break;
                case 4:
                    anchor.Share(userLists[0], userLists[1], userLists[2], userLists[3], OnShare);
                    break;
            }

            shareRequests.Add(new ShareRequest
            {
                AnchorSpace = anchor.Space.Handle,
                UserHandles = userLists.Select(user => user._handle).ToList()
            });
#pragma warning restore
        }

        await Task.Yield();

        ShareAnchorsHandle(shareRequests, expectedShareResult, numberOfQueries);

        Assert.AreEqual(0, ShareQueries.Count, "All shared queries should have been processed.");

        await Task.Yield();

        Assert.AreEqual(anchors.Count, shareResults.Count, "the number of result should equal the number of anchors");
    }

    public async Task ShareAnchorsUsingStatic(List<OVRSpatialAnchor> anchors, List<OVRSpaceUser> userList,
        Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult> shareResults,
        OVRPlugin.Result expectedShareCallbackResult, bool shareQueryResult, bool useLegacy)
    {
        void OnShare(ICollection<OVRSpatialAnchor> anchors, OVRSpatialAnchor.OperationResult result)
        {
            foreach (var anchor in anchors)
            {
                shareResults[anchor] = result;
            }
        }

        if (useLegacy)
        {
#pragma warning disable CS0618
            OVRSpatialAnchor.Share(anchors, userList, OnShare);
#pragma warning restore
        }
        else
        {
            OVRSpatialAnchor.ShareAsync(anchors, userList).ContinueWith(result =>
            {
                foreach (var anchor in anchors)
                {
                    shareResults[anchor] = result;
                }
            });
        }

        await Task.Yield();

        if (shareQueryResult)
        {
            Assert.AreEqual(1, ShareQueries.Count, "Only one share query is created.");

            var queryData = ShareQueries.First().Value;

            var queryUsers = new HashSet<ulong>(queryData.UserHandles.Select(handle => SpaceUsers[handle]));

            foreach (var anchor in anchors)
            {
#pragma warning disable CS0618
                Assert.IsTrue(queryData.Spaces.Contains(anchor.Space.Handle),
                    "anchor space should appear in the query space request");
#pragma warning restore
            }

            foreach (var user in userList)
            {
                Assert.IsTrue(queryData.UserHandles.Contains(user._handle),
                    "the user id should be appear in the share request");
            }

            var requestId = ShareQueries.First().Key;
            HandleShareQuery(requestId, expectedShareCallbackResult);
        }

        Assert.AreEqual(0, ShareQueries.Count, "All shared queries should have been processed.");

        await Task.Yield();

        Assert.AreEqual(anchors.Count, shareResults.Count, "the number of result should equal the number of anchors");
    }

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        CreateDefaultCameraRig();
        OVRObjectPool.Clear<List<OVRSpatialAnchor>>();
        OVRObjectPool.Clear<List<OVRSpaceUser>>();
    }

    private static int[] _anchorCountValues = { 0, 1, 5, 42, OVRPlugin.SpaceFilterInfoIdsMaxSize };

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator MaxAnchorCountAutomaticallySetAndNotRequired(
        [ValueSource(nameof(_anchorCountValues))]
        int count)
    {
        var uuids = Enumerable.Range(0, count).Select(_ => Guid.NewGuid()).ToArray();
#pragma warning disable CS0618
        OVRSpatialAnchor.LoadUnboundAnchors(new OVRSpatialAnchor.LoadOptions
#pragma warning restore
        {
            Uuids = uuids,
        }, Assert.IsNull);

        yield return null;

        Assert.AreEqual(1, SpaceQueries.Count);

        var (requestId, query) = SpaceQueries.First().Deconstruct();
        Assert.AreEqual(OVRPlugin.SpaceQueryFilterType.Ids, query.FilterType,
            "the filter type of the SpaceQuery option should be Ids");
        Assert.AreEqual(count, query.IdInfo.NumIds, ("the number of ids should be " + count));
        Assert.AreEqual(uuids, query.IdInfo.Ids.Take(count),
            "uuids of the SpaceQuery option should be the same than specified uuids");

        HandleSpaceQuery(requestId, null);

        yield return null;
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public void LoadUnboundAnchorsThrowsWhenMoreThanMaxCountAnchorsAreLoaded()
    {
        var count = OVRSpatialAnchor.LoadOptions.MaxSupported + 1;
#pragma warning disable CS0618
        Assert.Throws<ArgumentException>(() => OVRSpatialAnchor.LoadUnboundAnchors(
#pragma warning restore
            new OVRSpatialAnchor.LoadOptions
            {
                Uuids = Enumerable.Range(0, count).Select(_ => Guid.NewGuid()).ToArray(),
            }, Assert.IsNull), "Should output ArgumentException when specifying more than Max Uuids Supported.");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public void OVRSpaceQueryThrowsWhenMoreThanMaxCountUuidsAreAddedByArray()
    {
        var count = OVRSpaceQuery.Options.MaxUuidCount + 1;
        Assert.Throws<ArgumentException>(() => new OVRSpaceQuery.Options
        {
            UuidFilter = Enumerable.Range(0, count).Select(_ => Guid.NewGuid()).ToArray()
        }, "Should output ArgumentException when specifying more than Max Uuids Supported.");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public void OVRSpaceQueryThrowsWhenMoreThanMaxCountUuidsAreAddedByIEnumerable()
    {
        var count = OVRSpaceQuery.Options.MaxUuidCount + 1;
        Assert.Throws<InvalidOperationException>(() => new OVRSpaceQuery.Options
            {
                UuidFilter = Enumerable.Range(0, count).Select(_ => Guid.NewGuid())
            }.TryQuerySpaces(out _),
            "Should output InvalidOperationException when specifying more than Max Uuids Supported.");
    }

    [TestCase(false)]
    [TestCase(true)]
    [Timeout(DefaultTimeoutMs)]
    public void OVRSpaceQueryCorrectlyCopiesIds(bool asArray)
    {
        var count = OVRSpaceQuery.Options.MaxUuidCount;
        var guids = Enumerable.Range(0, count).Select(_ => Guid.NewGuid()).ToArray();

        IEnumerable<Guid> AsEnumerable()
        {
            foreach (var guid in guids)
            {
                yield return guid;
            }
        }

        new OVRSpaceQuery.Options
        {
            MaxResults = count,
            UuidFilter = asArray ? guids : AsEnumerable()
        }.TryQuerySpaces(out var requestId);

        var query = SpaceQueries[requestId];
        Assert.AreEqual(count, query.IdInfo.NumIds,
            "number of Ids in the option should be the same number of input uuids");
        Assert.AreEqual(guids, query.IdInfo.Ids.Take(count),
            "uuids in the option should be the same than the input uuids");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public void OVRSpaceQueryThrowsWhenTooManyIdsAreAddedLater()
    {
        var count = OVRSpaceQuery.Options.MaxUuidCount;
        var guids = Enumerable.Range(0, count).Select(_ => Guid.NewGuid()).ToList();
        var options = new OVRSpaceQuery.Options
        {
            MaxResults = count,
            UuidFilter = guids
        };

        // Now we add too many
        guids.Add(Guid.NewGuid());

        Assert.Throws<InvalidOperationException>(() => options.TryQuerySpaces(out _),
            "throw InvalidOperationException when UuidFilter is above Max count");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task CreateLocalizeAndDestroy_SingleAnchor_Success()
    {
        OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);

        Object.DestroyImmediate(anchor);
        Assert.Zero(Spaces.Count, "the number of spaces should be 0");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task CreateLocalizeAndDestroy_SingleAnchor_FailLocalizationCallback()
    {
        OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success,
            OVRPlugin.Result.Failure_SpaceLocalizationFailed, true);

        Object.DestroyImmediate(anchor);
        Assert.Zero(Spaces.Count, "the number of space should be 0");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task CreateLocalizeAndDestroy_SingleAnchor_FailLocalizationQuery()
    {
        LogAssert.ignoreFailingMessages = true;

        OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success,
            OVRPlugin.Result.Failure_SpaceLocalizationFailed, false);

        Object.DestroyImmediate(anchor);
        Assert.Zero(Spaces.Count, "the number of space should be 0");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task CreateLocalizeAndDestroy_SingleAnchor_FailCreationCallback()
    {
        OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Failure, OVRPlugin.Result.Success, false);

        Assert.IsFalse(anchor, "anchor should have been destroyed.");
        Assert.Zero(Spaces.Count, "the number of space should be 0");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task CreateLocalizeAndDestroy_SingleAnchor_FailCreationQuery()
    {
        LogAssert.ignoreFailingMessages = true;
        CreateSpatialAnchorSucceed = false;

        OVRSpatialAnchor anchor = new GameObject($"anchor").AddComponent<OVRSpatialAnchor>();

        // Need to wait 2 frames to the Creation call to occur
        await Task.Yield();
        await Task.Yield();

        Assert.IsFalse(anchor, "anchor should have been destroyed.");
        Assert.Zero(Spaces.Count, "the number of space should be 0");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task SaveAnchor_8AnchorsLocalAndCloud_Success()
    {
        const int nAnchors = 4;
        var localAnchors = new List<OVRSpatialAnchor>();
        var cloudAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            cloudAnchors.Add(anchor);
        }

        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        // Save
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(cloudAnchors[i], OVRSpace.StorageLocation.Cloud, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(cloudAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[cloudAnchors[i]], "the save result should be true as success");
        }

        foreach (var anchor in localAnchors.Union(cloudAnchors))
        {
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task SaveAnchor_8AnchorsLocalAndCloud_FailSaveCallback()
    {
        LogAssert.ignoreFailingMessages = true;
        const int nAnchors = 4;
        var localAnchors = new List<OVRSpatialAnchor>();
        var cloudAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            cloudAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Failure, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsFalse(savedResultByAnchor[localAnchors[i]], "the save result should be false as failure");
        }

        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(cloudAnchors[i], OVRSpace.StorageLocation.Cloud, savedResultByAnchor,
                OVRPlugin.Result.Failure, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(cloudAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsFalse(savedResultByAnchor[cloudAnchors[i]], "the save result should be false as failure");
        }

        foreach (var anchor in localAnchors.Union(cloudAnchors))
        {
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task SaveAnchorStatic_4Anchors_Success([Values] OVRSpace.StorageLocation storageLocation,
        [Values] bool useLegacy)
    {
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();
        var cloudAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Save
        var saveResults = await SaveAnchorStatic(anchors, storageLocation, OVRPlugin.Result.Success, saveQuerySuccess: true, useLegacy);
        Assert.AreEqual(anchors.Count, saveResults.Count,
            "the number of results should be equal to the number of anchors to save");
        foreach (var result in saveResults)
        {
            Assert.IsTrue(result.Value == OVRSpatialAnchor.OperationResult.Success,
                "The result of saving anchors is expected to be Success.");
        }

        foreach (var anchor in anchors.Union(cloudAnchors))
        {
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task SaveAnchorStatic_4Anchors_FailSaveQuery([Values] OVRSpace.StorageLocation storageLocation,
        [Values] bool useLegacy)
    {
        LogAssert.ignoreFailingMessages = true;
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();
        var cloudAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Save
        var saveResults = await SaveAnchorStatic(anchors, storageLocation, OVRPlugin.Result.Success, saveQuerySuccess: false, useLegacy);
        Assert.AreEqual(anchors.Count, saveResults.Count,
            "the number of results should be equal to the number of anchors to save");
        foreach (var result in saveResults)
        {
            Assert.IsTrue(result.Value == OVRSpatialAnchor.OperationResult.Failure,
                "The result of saving anchors is expected to be Failure.");
        }

        foreach (var anchor in anchors.Union(cloudAnchors))
        {
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task SaveAnchorStatic_4Anchors_FailSaveCallback([Values] OVRSpace.StorageLocation storageLocation,
        [Values] bool useLegacy)
    {
        LogAssert.ignoreFailingMessages = true;
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();
        var cloudAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Save
        var saveResults = await SaveAnchorStatic(anchors, storageLocation, OVRPlugin.Result.Failure, saveQuerySuccess: true, useLegacy);
        Assert.AreEqual(anchors.Count, saveResults.Count,
            "the number of results should be equal to the number of anchors to save");
        foreach (var result in saveResults)
        {
            Assert.IsTrue(result.Value == OVRSpatialAnchor.OperationResult.Failure,
                "The result of saving anchors is expected to be Failure.");
        }

        foreach (var anchor in anchors.Union(cloudAnchors))
        {
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public void SaveAnchorStatic_NullAnchors_ArgumentNullException([Values] OVRSpace.StorageLocation storageLocation,
        [Values] bool useLegacy)
    {
        // Save
        Assert.ThrowsAsync<ArgumentNullException>(
            async () => await SaveAnchorStatic(null, storageLocation, OVRPlugin.Result.Success, saveQuerySuccess: true, useLegacy),
            "Expected ArgumentNullException when a null argument is passed.");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task SaveAnchorStatic_EmptyAnchors_Success([Values] OVRSpace.StorageLocation storageLocation,
        [Values] bool useLegacy)
    {
        var anchors = new List<OVRSpatialAnchor>();

        // Save
        var saveResults = await SaveAnchorStatic(anchors, storageLocation, OVRPlugin.Result.Success, saveQuerySuccess: true, useLegacy);

        Assert.AreEqual(anchors.Count, saveResults.Count,
            "the number of results should be equal to the number of anchors to save");
    }


    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task Erase_4AnchorsLocalAndCloud_Success()
    {
        const int nAnchors = 4;
        var localAnchors = new List<OVRSpatialAnchor>();
        var cloudAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            cloudAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(cloudAnchors[i], OVRSpace.StorageLocation.Cloud, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(cloudAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[cloudAnchors[i]], "the save result should be true as success");
        }

        // Erase
        EraseSpaceSucceed = true;
        var eraseResults = new Dictionary<OVRSpatialAnchor, bool>();

        foreach (var anchor in localAnchors)
        {
            bool result = await EraseAnchors(anchor, OVRPlugin.SpaceStorageLocation.Local, OVRPlugin.Result.Success,
                true);
            Assert.IsTrue(result, "result of the Erase callback is true (successful)");
            eraseResults.Add(anchor, result);
        }

        foreach (var anchor in cloudAnchors)
        {
            bool result = await EraseAnchors(anchor, OVRPlugin.SpaceStorageLocation.Cloud, OVRPlugin.Result.Success,
                true);
            Assert.IsTrue(result, "result of the Erase callback is true (successful)");
            eraseResults.Add(anchor, result);
        }

        Assert.AreEqual(localAnchors.Count + cloudAnchors.Count, eraseResults.Count,
            "the number of erase result should be equal to the number of anchors to erase.");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task Erase_4AnchorsLocalAndCloud_FailEraseQuery()
    {
        LogAssert.ignoreFailingMessages = true;
        const int nAnchors = 4;
        var localAnchors = new List<OVRSpatialAnchor>();
        var cloudAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            cloudAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(cloudAnchors[i], OVRSpace.StorageLocation.Cloud, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(cloudAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[cloudAnchors[i]], "the save result should be true as success");
        }

        // Erase
        var eraseResults = new Dictionary<OVRSpatialAnchor, bool>();
        EraseSpaceSucceed = false;
        foreach (var anchor in localAnchors)
        {
            bool result = await EraseAnchors(anchor, OVRPlugin.SpaceStorageLocation.Local, OVRPlugin.Result.Success,
                false);
            Assert.IsFalse(result, "result of the Erase callback is false (failing)");
            eraseResults.Add(anchor, result);
        }

        foreach (var anchor in cloudAnchors)
        {
            bool result = await EraseAnchors(anchor, OVRPlugin.SpaceStorageLocation.Cloud, OVRPlugin.Result.Success,
                false);
            Assert.IsFalse(result, "result of the Erase callback is false (failing)");
            eraseResults.Add(anchor, result);
        }

        Assert.AreEqual(localAnchors.Count + cloudAnchors.Count, eraseResults.Count,
            "the number of erase result should be equal to the number of anchors to erase.");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task Erase_4AnchorsLocalAndCloud_FailEraseCallback()
    {
        const int nAnchors = 4;
        var localAnchors = new List<OVRSpatialAnchor>();
        var cloudAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            cloudAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(cloudAnchors[i], OVRSpace.StorageLocation.Cloud, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(cloudAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[cloudAnchors[i]], "the save result should be true as success");
        }

        // Erase
        var eraseResults = new Dictionary<OVRSpatialAnchor, bool>();
        EraseSpaceSucceed = true;
        foreach (var anchor in localAnchors)
        {
            bool result = await EraseAnchors(anchor, OVRPlugin.SpaceStorageLocation.Local, OVRPlugin.Result.Failure,
                true);
            Assert.IsFalse(result, "result of the Erase callback is false (failing)");
            eraseResults.Add(anchor, result);
        }

        foreach (var anchor in cloudAnchors)
        {
            bool result = await EraseAnchors(anchor, OVRPlugin.SpaceStorageLocation.Cloud, OVRPlugin.Result.Failure,
                true);
            Assert.IsFalse(result, "result of the Erase callback is false (failing)");
            eraseResults.Add(anchor, result);
        }

        Assert.AreEqual(localAnchors.Count + cloudAnchors.Count, eraseResults.Count,
            "the number of erase result should be equal to the number of anchors to erase.");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task LoadUnboundAnchors_4Anchors_Success([Values] OVRSpace.StorageLocation storageLocation,
        [Values] bool isASync)
    {
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(anchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor, OVRPlugin.Result.Success,
                true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(anchors[i]), "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[anchors[i]], "the save result should be true as success");
        }

        await Task.Yield();

        // Load
        List<Guid> loadUuids = new List<Guid>();

        for (int i = 0; i < anchors.Count; ++i)
        {
            loadUuids.Add(anchors[i].Uuid);
            GameObject.Destroy(anchors[i]);
        }

        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult;
        if (isASync)
        {
            unboundsAnchorsResult = await LoadAnchorsAsync(loadUuids, loadUuids, true);
        }
        else
        {
            unboundsAnchorsResult = await LoadAnchors(loadUuids, loadUuids, true);
        }

        await Task.Yield();

        Assert.AreEqual(4, unboundsAnchorsResult.Count, "there should be 4 anchors load result");
        foreach (var unboundAnchor in unboundsAnchorsResult)
        {
            Assert.IsTrue(loadUuids.Contains(unboundAnchor.Uuid),
                "loaded anchor should appear in the list of anchors to load");
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task LoadUnboundAnchors_4Anchors_WrongUuids([Values] OVRSpace.StorageLocation storageLocation,
        [Values] bool isAsync)
    {
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(anchors[i], storageLocation, savedResultByAnchor, OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(anchors[i]), "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[anchors[i]], "the save result should be true as success");
        }

        await Task.Yield();

        // Load
        List<Guid> loadUuids = new List<Guid>();
        List<Guid> ExisintgUuids = new List<Guid>();

        for (int i = 0; i < anchors.Count; ++i)
        {
            loadUuids.Add(Guid.NewGuid());
            ExisintgUuids.Add(anchors[i].Uuid);
            GameObject.Destroy(anchors[i]);
        }

        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult;
        if (isAsync)
        {
            unboundsAnchorsResult = await LoadAnchorsAsync(ExisintgUuids, loadUuids, true);
        }
        else
        {
            unboundsAnchorsResult = await LoadAnchors(ExisintgUuids, loadUuids, true);
        }

        await Task.Yield();

        Assert.AreEqual(0, unboundsAnchorsResult.Count, "there should be 0 anchors load result");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task LoadUnboundAnchors_4Anchors_FailSpaceQuery([Values] OVRSpace.StorageLocation storageLocation,
        [Values] bool isAsync)
    {
        LogAssert.ignoreFailingMessages = true;
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(anchors[i], storageLocation, savedResultByAnchor, OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(anchors[i]), "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[anchors[i]], "the save result should be true as success");
        }

        await Task.Yield();

        // Load
        SpaceQuerySucceed = false;
        List<Guid> loadUuids = new List<Guid>();

        for (int i = 0; i < anchors.Count; ++i)
        {
            loadUuids.Add(anchors[i].Uuid);
            GameObject.Destroy(anchors[i]);
        }

        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult;
        if (isAsync)
        {
            unboundsAnchorsResult = await LoadAnchorsAsync(loadUuids, loadUuids, false);
        }
        else
        {
            unboundsAnchorsResult = await LoadAnchors(loadUuids, loadUuids, false);
        }

        await Task.Yield();

        Assert.AreEqual(0, unboundsAnchorsResult.Count, "there should be 0 anchors load result");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task Localize_2LocalAnchorsAfterLoad_Success([Values] bool isASync)
    {
        const int nAnchors = 2;
        var localAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        // Load
        List<Guid> loadUuids = new List<Guid>();
        for (int i = 0; i < localAnchors.Count; ++i)
        {
            loadUuids.Add(localAnchors[i].Uuid);
            GameObject.Destroy(localAnchors[i]);
        }

        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult;
        if (isASync)
        {
            unboundsAnchorsResult = await LoadAnchorsAsync(loadUuids, loadUuids, true);
        }
        else
        {
            unboundsAnchorsResult = await LoadAnchors(loadUuids, loadUuids, true);
        }

        await Task.Yield();

        Assert.AreEqual(nAnchors, unboundsAnchorsResult.Count, "there should be " + nAnchors + " anchors load result");
        foreach (var unboundAnchor in unboundsAnchorsResult)
        {
            Assert.IsTrue(loadUuids.Contains(unboundAnchor.Uuid),
                "loaded anchor should appear in the list of anchors to load");
        }

        // Localize
        await LocalizeUnboundAnchors(unboundsAnchorsResult, OVRPlugin.Result.Success, unboundsAnchorsResult.Count);
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task Localize_2LocalAnchorsAfterLoad_FailLocalizeCallback([Values] bool isAsync)
    {
        const int nAnchors = 2;
        var localAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        // Load
        List<Guid> loadUuids = new List<Guid>();
        for (int i = 0; i < localAnchors.Count; ++i)
        {
            loadUuids.Add(localAnchors[i].Uuid);
            GameObject.Destroy(localAnchors[i]);
        }

        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult;
        if (isAsync)
        {
            unboundsAnchorsResult = await LoadAnchorsAsync(loadUuids, loadUuids, true);
        }
        else
        {
            unboundsAnchorsResult = await LoadAnchors(loadUuids, loadUuids, true);
        }

        await Task.Yield();

        Assert.AreEqual(nAnchors, unboundsAnchorsResult.Count, "there should be " + nAnchors + " anchors load result");
        foreach (var unboundAnchor in unboundsAnchorsResult)
        {
            Assert.IsTrue(loadUuids.Contains(unboundAnchor.Uuid),
                "loaded anchor should appear in the list of anchors to load");
        }

        // Localize
        await LocalizeUnboundAnchors(unboundsAnchorsResult, OVRPlugin.Result.Failure, 0);
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task BindTo_2Anchors([Values] OVRSpace.StorageLocation storageLocation, [Values] bool isAsync)
    {
        const int nAnchors = 2;
        var localAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        // Load
        List<Guid> loadUuids = new List<Guid>();
        for (int i = 0; i < localAnchors.Count; ++i)
        {
            loadUuids.Add(localAnchors[i].Uuid);
            GameObject.Destroy(localAnchors[i]);
        }

        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult;
        if (isAsync)
        {
            unboundsAnchorsResult = await LoadAnchorsAsync(loadUuids, loadUuids, true);
        }
        else
        {
            unboundsAnchorsResult = await LoadAnchors(loadUuids, loadUuids, true);
        }

        await Task.Yield();

        Assert.AreEqual(nAnchors, unboundsAnchorsResult.Count, "there should be " + nAnchors + " anchors load result");
        foreach (var unboundAnchor in unboundsAnchorsResult)
        {
            Assert.IsTrue(loadUuids.Contains(unboundAnchor.Uuid),
                "loaded anchor should appear in the list of anchors to load");
        }

        // Localize
        await LocalizeUnboundAnchors(unboundsAnchorsResult, OVRPlugin.Result.Success, unboundsAnchorsResult.Count);

        // BindTo
        foreach (var unboundAnchor in unboundsAnchorsResult)
        {
            OVRSpatialAnchor spatialAnchor = new GameObject().AddComponent<OVRSpatialAnchor>();
            spatialAnchor.gameObject.transform.SetPositionAndRotation(unboundAnchor.Pose.position,
                unboundAnchor.Pose.rotation);
            unboundAnchor.BindTo(spatialAnchor);

            await Task.Yield();

            Assert.IsTrue(spatialAnchor);
            Assert.IsTrue(spatialAnchor.Localized);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task BindTo_ExistingAnchor([Values] OVRSpace.StorageLocation storageLocation)
    {
        const int nAnchors = 2;
        var localAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        // Load
        List<Guid> loadUuids = new List<Guid>();
        for (int i = 0; i < localAnchors.Count; ++i)
        {
            loadUuids.Add(localAnchors[i].Uuid);
            GameObject.Destroy(localAnchors[i]);
        }

        localAnchors.Clear();

        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult = await LoadAnchors(loadUuids, loadUuids, true);
        await Task.Yield();

        Assert.AreEqual(nAnchors, unboundsAnchorsResult.Count, "there should be " + nAnchors + " anchors load result");
        foreach (var unboundAnchor in unboundsAnchorsResult)
        {
            Assert.IsTrue(loadUuids.Contains(unboundAnchor.Uuid),
                "loaded anchor should appear in the list of anchors to load");
        }

        // Localize
        await LocalizeUnboundAnchors(unboundsAnchorsResult, OVRPlugin.Result.Success, unboundsAnchorsResult.Count);

        // Create new Anchors
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        // BindTo
        for (int i = 0; i < unboundsAnchorsResult.Count; ++i)
        {
            Assert.IsTrue(localAnchors[i].Created, "the anchor should be created");
            Assert.Throws<ArgumentException>(() => unboundsAnchorsResult[i].BindTo(localAnchors[i]),
                "Expected to throw ArgumentException because the anchor is already created");
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task BindTo_AnchorCreationPending([Values] OVRSpace.StorageLocation storageLocation,
        [Values] bool isAsync)
    {
        const int nAnchors = 2;
        var localAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        // Load
        List<Guid> loadUuids = new List<Guid>();
        for (int i = 0; i < localAnchors.Count; ++i)
        {
            loadUuids.Add(localAnchors[i].Uuid);
            GameObject.Destroy(localAnchors[i]);
        }

        localAnchors.Clear();

        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult;
        if (isAsync)
        {
            unboundsAnchorsResult = await LoadAnchorsAsync(loadUuids, loadUuids, true);
        }
        else
        {
            unboundsAnchorsResult = await LoadAnchors(loadUuids, loadUuids, true);
        }

        await Task.Yield();

        Assert.AreEqual(nAnchors, unboundsAnchorsResult.Count, "there should be " + nAnchors + " anchors load result");
        foreach (var unboundAnchor in unboundsAnchorsResult)
        {
            Assert.IsTrue(loadUuids.Contains(unboundAnchor.Uuid),
                "loaded anchor should appear in the list of anchors to load");
        }

        // Localize
        await LocalizeUnboundAnchors(unboundsAnchorsResult, OVRPlugin.Result.Success, unboundsAnchorsResult.Count);

        // Create new Anchors
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = new GameObject($"anchor").AddComponent<OVRSpatialAnchor>();
            localAnchors.Add(anchor);
        }

        await Task.Yield();

        // BindTo
        for (int i = 0; i < unboundsAnchorsResult.Count; ++i)
        {
            Assert.IsFalse(localAnchors[i].Created, "anchor should not be created.");
            Assert.IsTrue(localAnchors[i].PendingCreation, "anchor should be in the PendingCreation state");
            Assert.Throws<ArgumentException>(() => unboundsAnchorsResult[i].BindTo(localAnchors[i]),
                "Expected to throw ArgumentException because the anchor is use for creation");
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task BindTo_Null([Values] bool isAsync)
    {
        const int nAnchors = 2;
        var localAnchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            localAnchors.Add(anchor);
        }

        // Save
        Dictionary<OVRSpatialAnchor, bool> savedResultByAnchor = new Dictionary<OVRSpatialAnchor, bool>();
        for (var i = 0; i < nAnchors; i++)
        {
            await SaveAnchor(localAnchors[i], OVRSpace.StorageLocation.Local, savedResultByAnchor,
                OVRPlugin.Result.Success, true);
            Assert.IsTrue(savedResultByAnchor.ContainsKey(localAnchors[i]),
                "the result should contains the anchor to save");
            Assert.IsTrue(savedResultByAnchor[localAnchors[i]], "the save result should be true as success");
        }

        // Load
        List<Guid> loadUuids = new List<Guid>();
        for (int i = 0; i < localAnchors.Count; ++i)
        {
            loadUuids.Add(localAnchors[i].Uuid);
            GameObject.Destroy(localAnchors[i]);
        }

        localAnchors.Clear();

        List<OVRSpatialAnchor.UnboundAnchor> unboundsAnchorsResult;
        if (isAsync)
        {
            unboundsAnchorsResult = await LoadAnchorsAsync(loadUuids, loadUuids, true);
        }
        else
        {
            unboundsAnchorsResult = await LoadAnchors(loadUuids, loadUuids, true);
        }

        await Task.Yield();

        Assert.AreEqual(nAnchors, unboundsAnchorsResult.Count, "there should be " + nAnchors + " anchors load result");
        foreach (var unboundAnchor in unboundsAnchorsResult)
        {
            Assert.IsTrue(loadUuids.Contains(unboundAnchor.Uuid),
                "loaded anchor should appear in the list of anchors to load");
        }

        // Localize
        await LocalizeUnboundAnchors(unboundsAnchorsResult, OVRPlugin.Result.Success, unboundsAnchorsResult.Count);

        // BindTo
        for (int i = 0; i < unboundsAnchorsResult.Count; ++i)
        {
            Assert.Throws<ArgumentNullException>(() => unboundsAnchorsResult[i].BindTo(null));
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task ShareList_4Anchors_Success()
    {
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Share
        var users = Enumerable.Range(1, 4).Select(x => new OVRSpaceUser((ulong)x)).ToArray();

        var userLists = new List<OVRSpaceUser[]>()
        {
            new[] { users[0], users[1] },
            new[] { users[1], users[0] },
            new[] { users[2], users[3] },
            new[] { users[3], users[2] },
        };

        var shareResults = new Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>();

        await ShareAnchorsUsingList(anchors, userLists, shareResults, OVRPlugin.Result.Success, 2, ShareSpaceSucceed);


        foreach (var anchor in anchors)
        {
            Assert.AreEqual(OVRSpatialAnchor.OperationResult.Success, shareResults[anchor],
                "the anchor share is expceted to succeed.");
            Object.DestroyImmediate(anchor);
        }
    }

    // true if load from local memory
    // false if load from the cloud
    protected static readonly object[] numberOfUsers =
    {
        new object[] { 1 },
        new object[] { 2 },
        new object[] { 3 },
        new object[] { 4 }
    };

    [Test]
    [Timeout(DefaultTimeoutMs)]
    [TestCaseSource(nameof(numberOfUsers))]
    public async Task ShareToXUsers_4Anchors_Success(int numberOfUsers)
    {
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Share
        var users = Enumerable.Range(1, numberOfUsers).Select(x => new OVRSpaceUser((ulong)x)).ToList();

        var shareResults = new Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>();

        await ShareAnchors(anchors, users, shareResults, OVRPlugin.Result.Success, 1);


        foreach (var anchor in anchors)
        {
            Assert.AreEqual(OVRSpatialAnchor.OperationResult.Success, shareResults[anchor],
                "the anchor share is expceted to succeed.");
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task ShareList_4Anchors_FailShareCallback()
    {
        LogAssert.ignoreFailingMessages = true;
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Share
        var users = Enumerable.Range(1, 4).Select(x => new OVRSpaceUser((ulong)x)).ToArray();

        var userLists = new List<OVRSpaceUser[]>()
        {
            new[] { users[0], users[1] },
            new[] { users[1], users[0] },
            new[] { users[2], users[3] },
            new[] { users[3], users[2] },
        };

        var shareResults = new Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>();
        await ShareAnchorsUsingList(anchors, userLists, shareResults,
            OVRPlugin.Result.Failure_SpaceCloudStorageDisabled, 2, ShareSpaceSucceed);

        foreach (var anchor in anchors)
        {
            Assert.AreEqual(OVRSpatialAnchor.OperationResult.Failure_SpaceCloudStorageDisabled, shareResults[anchor],
                "the anchor share is expceted to fail.");
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task ShareStatic_4Anchors_Success([Values] bool useLegacy)
    {
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Share
        var users = Enumerable.Range(1, 4).Select(x => new OVRSpaceUser((ulong)x)).ToList();

        var shareResults = new Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>();

        await ShareAnchorsUsingStatic(anchors, users, shareResults, OVRPlugin.Result.Success,
            shareQueryResult: true, useLegacy: useLegacy);

        foreach (var anchor in anchors)
        {
            Assert.AreEqual(OVRSpatialAnchor.OperationResult.Success, shareResults[anchor],
                "the anchor share is expceted to succeed.");
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task ShareStatic_4Anchors_FailQuery([Values] bool useLegacy)
    {
        LogAssert.ignoreFailingMessages = true;
        ShareSpaceSucceed = false;

        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Share
        var users = Enumerable.Range(1, 4).Select(x => new OVRSpaceUser((ulong)x)).ToList();

        var shareResults = new Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>();

        await ShareAnchorsUsingStatic(anchors, users, shareResults, OVRPlugin.Result.Failure,
            ShareSpaceSucceed, useLegacy: useLegacy);

        foreach (var anchor in anchors)
        {
            Assert.AreEqual(OVRSpatialAnchor.OperationResult.Failure, shareResults[anchor],
                "the anchor share is expceted to fail.");
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task ShareStatic_4Anchors_FailCallback([Values] bool useLegacy)
    {
        LogAssert.ignoreFailingMessages = true;

        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Share
        var users = Enumerable.Range(1, 4).Select(x => new OVRSpaceUser((ulong)x)).ToList();

        var shareResults = new Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>();

        await ShareAnchorsUsingStatic(anchors, users, shareResults, OVRPlugin.Result.Failure_SpaceCloudStorageDisabled,
            ShareSpaceSucceed, useLegacy: useLegacy);

        foreach (var anchor in anchors)
        {
            Assert.AreEqual(OVRSpatialAnchor.OperationResult.Failure_SpaceCloudStorageDisabled, shareResults[anchor],
                "the anchor share is expected to fail.");
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public void ShareStatic_NullAnchors_ArgumentNullException()
    {
        // Share
        var users = Enumerable.Range(1, 4).Select(x => new OVRSpaceUser((ulong)x)).ToList();

#pragma warning disable CS0618
        Assert.Throws<ArgumentNullException>(() => OVRSpatialAnchor.Share(null, users),
            "ArgumentNullException should be thrown when anchors parameter is null.");
#pragma warning restore
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task ShareStatic_EmptyUsers_Success([Values] bool useLegacy)
    {
        const int nAnchors = 4;
        var anchors = new List<OVRSpatialAnchor>();

        // Create
        for (var i = 0; i < nAnchors; i++)
        {
            OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
            anchors.Add(anchor);
        }

        // Share
        var users = new List<OVRSpaceUser>();

        var shareResults = new Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>();

        await ShareAnchorsUsingStatic(anchors, users, shareResults, OVRPlugin.Result.Success, shareQueryResult: true, useLegacy: useLegacy);

        await Task.Yield();

        Assert.AreEqual(anchors.Count, shareResults.Count,
            "The number or result should be the same than the number of anchors.");

        foreach (var anchor in anchors)
        {
            Assert.AreEqual(OVRSpatialAnchor.OperationResult.Success, shareResults[anchor],
                "the anchor share should succeed.");
            Object.DestroyImmediate(anchor);
        }
    }


    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task ShareStatic_EmptyAnchors_Success([Values] bool useLegacy)
    {
        var anchors = new List<OVRSpatialAnchor>();

        // Share
        var users = Enumerable.Range(1, 4).Select(x => new OVRSpaceUser((ulong)x)).ToList();

        var shareResults = new Dictionary<OVRSpatialAnchor, OVRSpatialAnchor.OperationResult>();

        await ShareAnchorsUsingStatic(anchors, users, shareResults, OVRPlugin.Result.Success,
            shareQueryResult: true, useLegacy: useLegacy);

        await Task.Yield();

        Assert.AreEqual(anchors.Count, shareResults.Count,
            "The number or result should be the same than the number of anchors.");

        foreach (var anchor in anchors)
        {
            Assert.AreEqual(OVRSpatialAnchor.OperationResult.Success, shareResults[anchor],
                "the anchor share should succeed.");
            Object.DestroyImmediate(anchor);
        }
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task InitializeFromExisting_ExistingAnchor_InvalidOperationException()
    {
        // Create
        OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);

        var anchor2nd = new GameObject($"anchor").AddComponent<OVRSpatialAnchor>();
#pragma warning disable CS0618
        Assert.Throws<InvalidOperationException>(() => anchor2nd.InitializeFromExisting(anchor.Space, anchor.Uuid),
            "Exception should be thrown because Anchor already exists");
#pragma warning restore
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task InitializeFromExisting_1Anchor_Success()
    {
        // Create
        OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
        Guid uuid = anchor.Uuid;

        GameObject.Destroy(anchor.gameObject);

        await Task.Yield();

        // Create
        var space = new SpatialAnchor(uuid);
        AddSpace(space);

        // Initialize from Existing
        var anchor2nd = await CreateAnchorUsingInitializeFromExisting(uuid, space, OVRPlugin.Result.Success, true);

        await Task.Yield();
        await Task.Yield();

        Assert.IsTrue(anchor2nd.Localized, "anchor should be localized");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task InitializeFromExisting_1Anchor_FailLocalizationCallback()
    {
        LogAssert.ignoreFailingMessages = true;
        // Create
        OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
        Guid uuid = anchor.Uuid;

        GameObject.Destroy(anchor.gameObject);

        await Task.Yield();

        // Create
        var space = new SpatialAnchor(uuid);
        AddSpace(space);

        // Initialize from Existing
        var anchor2nd = await CreateAnchorUsingInitializeFromExisting(uuid, space, OVRPlugin.Result.Failure, true);

        await Task.Yield();
        await Task.Yield();

        Assert.IsFalse(anchor2nd.Localized, "anchor should be localized");
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task InitializeFromExisting_1Anchor_FailLocalizationQuery()
    {
        // Create
        OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
        Guid uuid = anchor.Uuid;

        GameObject.Destroy(anchor.gameObject);

        await Task.Yield();

        // Create
        var space = new SpatialAnchor(uuid);
        AddSpace(space);

        // Initialize from Existing
        var anchor2nd = await CreateAnchorUsingInitializeFromExisting(uuid, space, OVRPlugin.Result.Success, false);

        await Task.Yield();
        await Task.Yield();

        Assert.IsFalse(anchor2nd.Localized, "anchor should be localized");
    }


    private static Type[] _anchorTypes = { typeof(OVRSpatialAnchor), typeof(OVRSceneAnchor) };

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CameraRigDisablesAnchorsWhenTheyAreAParent(
        [ValueSource(nameof(_anchorTypes))] Type anchorType)
    {
        var rig = Object.FindAnyObjectByType<OVRCameraRig>();
        var anchor = new GameObject("Anchor", anchorType)
            .GetComponent(anchorType) as MonoBehaviour;
        Assert.True(anchor);

        // Let the anchor's OnEnable run; so far no errors
        yield return null;
        Assert.True(anchor.enabled);

        // Now change the hierarchy to make it invalid
        rig.transform.parent = anchor.transform;

        LogAssert.Expect(LogType.Error,
            $"The {anchorType.Name} '{anchor.name}' is a parent of the {nameof(OVRCameraRig)} '{rig.name}', which is not allowed. An {anchorType.Name} may not be the parent of an {nameof(OVRCameraRig)} because the {nameof(OVRCameraRig)} defines the tracking space for the anchor, and its transform is relative to the {nameof(OVRCameraRig)}.");

        yield return null;

        // Anchor should be disabled now.
        Assert.False(anchor.enabled);
    }


    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task CanEraseAsync([Values] bool eraseResult)
    {
        // Create
        OVRSpatialAnchor anchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
        Guid uuid = anchor.Uuid;

        await Task.Yield();

        AddSpace(new SpatialAnchor(uuid));

        await Task.Run(async () =>
        {
            while (anchor.PendingCreation) await Task.Yield();
        });

        var task = anchor.EraseAsync();
        Assert.IsFalse(task.IsCompleted);
        await Task.Yield();

        Assert.AreEqual(1, EraseQueries.Count);
        var (requestId, space) = EraseQueries.Select(pair => (pair.Key, pair.Value)).ToArray()[0];

        HandleEraseQuery(requestId, eraseResult ? OVRPlugin.Result.Success : OVRPlugin.Result.Failure, uuid,
            OVRPlugin.SpaceStorageLocation.Local);
        await Task.Yield();

        Assert.IsTrue(task.IsCompleted);
        Assert.AreEqual(eraseResult, task.GetResult());

        await Task.Yield();
        Object.DestroyImmediate(anchor);
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public void OVRAnchorCanBeRetrievedAsUnboundAnchor()
    {
        var space = CreateSpace(OvrSpaceFBType.spatial_anchor);
        var anchor = new OVRAnchor(space.Handle, space.Uuid);
        var result = OVRSpatialAnchor.FromOVRAnchor(anchor, out var unboundAnchor);

        Assert.That(result, Is.True);
        Assert.That(anchor.Uuid, Is.EqualTo(unboundAnchor.Uuid));
        Assert.That(anchor.Handle, Is.EqualTo(unboundAnchor._space.Handle));
    }

    [Test]
    [Timeout(DefaultTimeoutMs)]
    public async Task OVRAnchorCannotBeRetrievedAsUnboundAnchorIfAlreadyBound()
    {
        var spatialAnchor = await CreateAnchor(OVRPlugin.Result.Success, OVRPlugin.Result.Success, true);
        Assert.That(spatialAnchor.Created, Is.True);

        var anchor = new OVRAnchor(spatialAnchor._anchor.Handle, spatialAnchor.Uuid);
        Assert.That(OVRSpatialAnchor.FromOVRAnchor(anchor, out _), Is.False);
    }

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_spatial_entity_discovery
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator LoadUnboundAnchorsAsyncWithIncrementalResults()
    {
        // Add spatial anchors to the mock's anchor store
        var discoveryResults = Enumerable.Range(0, 10).Select(i =>
        {
            var anchor = new SpatialAnchor();
            AddSpace(anchor);
            return new OVRPlugin.SpaceDiscoveryResult
            {
                Space = anchor.Handle,
                Uuid = anchor.Uuid
            };
        }).ToArray();

        // Try to load them
        var unboundAnchors = new List<OVRSpatialAnchor.UnboundAnchor>();
        var boundAnchors = new List<OVRSpatialAnchor>();
        int? lastStartingIndex = null;
        var desiredBindCount = 0;
        OVRPlugin.SpaceDiscoveryResult[] expectedPartialResults = null;

        var task = OVRSpatialAnchor.LoadUnboundAnchorsAsync(discoveryResults.Select(r => r.Uuid), unboundAnchors, (partialResults, startingIndex) =>
        {
            lastStartingIndex = startingIndex;

            // Asserts those newly added anchors are correct
            Assert.That(expectedPartialResults, Is.Not.Null);
            Assert.That(partialResults.Skip(startingIndex),
                Is.EquivalentTo(expectedPartialResults.Select(r => new OVRSpatialAnchor.UnboundAnchor(r.Space, r.Uuid))));

            // Generate a list of random indices to bind
            var count = partialResults.Count;
            var indices = Enumerable.Range(0, count).ToArray();
            foreach (var _ in indices)
            {
                Swap(indices, Random.Range(0, count), Random.Range(0, count));
            }

            foreach (var index in indices.Take(desiredBindCount))
            {
                Bind(unboundAnchors[index]);
            }
        });

        var (requestId, _) = DiscoveryQueries.First(kvp =>
        {
            var filters = kvp.Value.Filters;
            if (filters.Length != 1) return false;
            if (filters[0] is not DiscoveryFilterInfoIds idFilter) return false;
            return discoveryResults.Select(r => r.Uuid).All(idFilter.Ids.Contains);
        });

        var nextStartingIndex = 0;
        IEnumerator AssertIncrementalResults(int take, int bindCount)
        {
            desiredBindCount = bindCount;
            var skip = nextStartingIndex;
            expectedPartialResults = discoveryResults.Skip(skip).Take(take).ToArray();
            nextStartingIndex += expectedPartialResults.Length;

            // Put the result into the event queue
            AddSpaceDiscoveryResults(requestId, expectedPartialResults, isFinal: skip + take >= discoveryResults.Length);
            yield return null;

            // Asserts the callback was invoked
            Assert.That(lastStartingIndex.HasValue);

            // Reset for the next time
            lastStartingIndex = null;
        }

        void Swap<T>(T[] array, int indexA, int indexB)
            => (array[indexA], array[indexB]) = (array[indexB], array[indexA]);

        void Bind(OVRSpatialAnchor.UnboundAnchor unboundAnchor)
        {
            var boundAnchor = new GameObject($"Anchor {unboundAnchor.Uuid}").AddComponent<OVRSpatialAnchor>();
            unboundAnchor.BindTo(boundAnchor);
            boundAnchors.Add(boundAnchor);
        }

        // Provide partial results
        yield return AssertIncrementalResults(take: 4, bindCount: 2);
        Assert.That(unboundAnchors.Count, Is.EqualTo(4)); // we provided 4 new anchors
        Assert.That(boundAnchors.Count, Is.EqualTo(2));   // and bound two of them

        // Provide more results
        yield return AssertIncrementalResults(take: 4, bindCount: 2);
        Assert.That(unboundAnchors.Count, Is.EqualTo(6)); // 8 have been provided, but we bound 2 in the first
                                                          // incremental results callback, so those should have been
                                                          // removed on the invocation of the second callback.
        Assert.That(boundAnchors.Count, Is.EqualTo(4));   // 4 total have been bound

        // If a batch contains already bound anchors, then we shouldn't get a callback at all
        AddSpaceDiscoveryResults(requestId, boundAnchors.Select(anchor => new OVRPlugin.SpaceDiscoveryResult
        {
            Space = anchor._anchor.Handle,
            Uuid = anchor.Uuid,
        }).ToArray(), isFinal: false);
        yield return null;

        Assert.That(lastStartingIndex.HasValue, Is.False);

        // Final batch
        yield return AssertIncrementalResults(take: 2, bindCount: 1);
        Assert.That(task.IsCompleted);
        var result = task.GetResult();
        Assert.That(result.Success);
        Assert.That(result.Value, Is.EqualTo(unboundAnchors));
        Assert.That(unboundAnchors.Count, Is.EqualTo(5)); // Since this was the last call, we expect the final list to
                                                          // only contain anchors we haven't yet bound.
        Assert.That(boundAnchors.Count, Is.EqualTo(5));

        // Bound and unbound anchor sets should be disjoint
        Assert.That(boundAnchors.Select(a => a.Uuid).Intersect(unboundAnchors.Select(a => a.Uuid)), Is.Empty);
    }
#endif // XR_META_spatial_entity_discovery
}

#endif // OVRPLUGIN_TESTING
