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
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.TestTools;
using Random = UnityEngine.Random;

[TestFixture]
internal class OVRAnchorPlayModeTests : AnchorTestFixture
{
    private GameObject _ovrManagerObject;
    private OVRManager _ovrManager;
    private Camera _ovrCamera;
    private Space _expectedSpace;
    private Space _unsupportedSpace;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        _ovrManagerObject = new GameObject("OvrManager");
        _ovrManager = _ovrManagerObject.AddComponent<OVRManager>();
        _ovrCamera = _ovrManagerObject.AddComponent<Camera>();
        SetSeededRandomPositionAndRotation(_ovrManagerObject);
        yield return BaseSetup();
        yield return WaitForOVRManager();
    }

    private void SetSeededRandomPositionAndRotation(GameObject gameObject)
    {
        UnityEngine.Random.InitState(0);
        gameObject.transform.SetPositionAndRotation(UnityEngine.Random.insideUnitSphere,
            UnityEngine.Random.rotationUniform);
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        UnityEngine.Object.Destroy(_ovrManagerObject);
        yield return base.UnityTearDown();

#if OVR_INTERNAL_CODE // Discovery
        OVRTask<OVRResult<List<OVRAnchor>, OVRAnchor.FetchResult>>.Clear();
#endif
    }

    [UnityTest]
    public IEnumerator TestFetchAsyncByComponentType()
    {
        _expectedSpace.AddComponent<SpaceContainer>();

        var list = OVRObjectPool.Get<List<OVRAnchor>>();
        var task = OVRAnchor.FetchAnchorsAsync(OVRPlugin.SpaceComponentType.SpaceContainer, list);

        yield return WaitForTaskCompletion(task, HandleAllSpaceQueries);

        var result = task.GetResult();
        Assert.IsTrue(result);
        Assert.AreEqual(1, list.Count);
        var anchor = list[0];
        Assert.AreNotEqual(anchor, OVRAnchor.Null);
        Assert.AreEqual(_expectedSpace.Uuid, anchor.Uuid);
        OVRObjectPool.Return(list);
    }

    [UnityTest]
    public IEnumerator TestFetchAsyncByUuids()
    {
        var list = OVRObjectPool.Get<List<OVRAnchor>>();
        var task = OVRAnchor.FetchAnchorsAsync(new Guid[] { _expectedSpace.Uuid }, list);

        yield return WaitForTaskCompletion(task, HandleAllSpaceQueries);

        var result = task.GetResult();
        Assert.IsTrue(result);
        Assert.AreEqual(1, list.Count);
        var anchor = list[0];
        Assert.AreNotEqual(anchor, OVRAnchor.Null);
        Assert.AreEqual(_expectedSpace.Uuid, anchor.Uuid);
        OVRObjectPool.Return(list);
    }

#if OVR_INTERNAL_CODE
    [UnityTest]
    public IEnumerator SingleAnchorCanBeSaved()
    {
        var creationTask = OVRAnchor.CreateSpatialAnchorAsync(Pose.identity);
        yield return WaitForTaskCompletion(creationTask, HandleAllSpatialAnchorCreationRequests);
        var anchor = creationTask.GetResult();

        var saveTask = anchor.SaveAsync();
        Assert.That(saveTask.IsPending, Is.True);

        var expectedResult = (OVRAnchor.SaveResult)1234;

        yield return WaitForTaskCompletion(saveTask, () =>
        {
            Assert.That(SaveSpacesRequests.Count, Is.EqualTo(1));
            var (requestId, spaces) = SaveSpacesRequests.First();
            Assert.That(spaces, Is.EqualTo(new[] {anchor.Handle}));
            EnqueueEvent(OVRPlugin.EventType.SpacesSaveResult, new OVRDeserialize.SpacesSaveResultData
            {
                RequestId = requestId,
                Result = expectedResult
            });
            SaveSpacesRequests.Clear();
        });

        Assert.That(saveTask.GetResult(), Is.EqualTo(OVRResult.From(expectedResult)));
    }

    [UnityTest]
    public IEnumerator MultipleAnchorsCanBeSaved()
    {
        var creationTasks = new []
        {
            OVRAnchor.CreateSpatialAnchorAsync(Pose.identity),
            OVRAnchor.CreateSpatialAnchorAsync(Pose.identity),
            OVRAnchor.CreateSpatialAnchorAsync(Pose.identity),
            OVRAnchor.CreateSpatialAnchorAsync(Pose.identity),
        };

        HandleAllSpatialAnchorCreationRequests();
        yield return null;

        var anchors = creationTasks.Select(t => t.GetResult()).ToArray();
        var saveTask = OVRAnchor.SaveAsync(anchors);

        Assert.That(saveTask.IsPending, Is.True);

        var expectedResult = (OVRAnchor.SaveResult)1234;

        yield return WaitForTaskCompletion(saveTask, () =>
        {
            Assert.That(SaveSpacesRequests.Count, Is.EqualTo(1));
            var (requestId, spaces) = SaveSpacesRequests.First();
            Assert.That(spaces, Is.EqualTo(anchors.Select(anchor => anchor.Handle)));
            EnqueueEvent(OVRPlugin.EventType.SpacesSaveResult, new OVRDeserialize.SpacesSaveResultData
            {
                RequestId = requestId,
                Result = expectedResult
            });
            SaveSpacesRequests.Clear();
        });

        Assert.That(saveTask.GetResult(), Is.EqualTo(OVRResult.From(expectedResult)));
    }

    [UnityTest]
    public IEnumerator SingleAnchorCanBeErased()
    {
        var creationTask = OVRAnchor.CreateSpatialAnchorAsync(Pose.identity);
        yield return WaitForTaskCompletion(creationTask, HandleAllSpatialAnchorCreationRequests);
        var anchor = creationTask.GetResult();

        var eraseTask = anchor.EraseAsync();
        Assert.That(eraseTask.IsPending, Is.True);

        var expectedResult = (OVRAnchor.EraseResult)1234;

        yield return WaitForTaskCompletion(eraseTask, () =>
        {
            Assert.That(EraseSpacesRequests.Count, Is.EqualTo(1));
            var (requestId, info) = EraseSpacesRequests.First();
            Assert.That(info.Spaces, Is.EqualTo(new[] {anchor.Handle}));
            EnqueueEvent(OVRPlugin.EventType.SpacesEraseResult, new OVRDeserialize.SpacesEraseResultData
            {
                RequestId = requestId,
                Result = expectedResult
            });
            EraseSpacesRequests.Clear();
        });

        Assert.That(eraseTask.GetResult(), Is.EqualTo(OVRResult.From(expectedResult)));
    }

    [UnityTest]
    public IEnumerator MultipleAnchorsCanBeErased()
    {
        var creationTasks = new []
        {
            OVRAnchor.CreateSpatialAnchorAsync(Pose.identity),
            OVRAnchor.CreateSpatialAnchorAsync(Pose.identity),
            OVRAnchor.CreateSpatialAnchorAsync(Pose.identity),
            OVRAnchor.CreateSpatialAnchorAsync(Pose.identity),
        };

        HandleAllSpatialAnchorCreationRequests();
        yield return null;

        var anchors = creationTasks.Select(t => t.GetResult()).ToArray();

        // 4 anchors total -- erase 2 by handle and 2 by uuid
        var eraseTask = OVRAnchor.EraseAsync(anchors.Take(2), anchors.Skip(2).Select(a => a.Uuid));

        Assert.That(eraseTask.IsPending, Is.True);

        var expectedResult = (OVRAnchor.EraseResult)1234;

        yield return WaitForTaskCompletion(eraseTask, () =>
        {
            Assert.That(EraseSpacesRequests.Count, Is.EqualTo(1));
            var (requestId, info) = EraseSpacesRequests.First();

            // The first 2 are erased by XrSpace handle
            Assert.That(info.Spaces, Is.EqualTo(anchors.Take(2).Select(anchor => anchor.Handle)));

            // The second 2 are erased by UUID
            Assert.That(info.Uuids, Is.EqualTo(anchors.Skip(2).Select(anchor => anchor.Uuid)));

            EnqueueEvent(OVRPlugin.EventType.SpacesEraseResult, new OVRDeserialize.SpacesEraseResultData
            {
                RequestId = requestId,
                Result = expectedResult
            });
            EraseSpacesRequests.Clear();
        });

        Assert.That(eraseTask.GetResult(), Is.EqualTo(OVRResult.From(expectedResult)));
    }

    [Test]
    public void FetchAnchorsWithNullAnchorListThrowsNullArgumentException()
    {
        Assert.Throws<ArgumentNullException>(() =>
        {
            OVRAnchor.FetchAnchorsAsync(null, default(OVRAnchor.FetchOptions));
        });
    }

    [UnityTest]
    public IEnumerator SpaceDiscoveryCompletionEventIsFired()
    {
        OVRResult<List<OVRAnchor>, OVRAnchor.FetchResult>? actualResult = null;

        var list = new List<OVRAnchor>();
        var originalTask = OVRAnchor.FetchAnchorsAsync(list, new OVRAnchor.FetchOptions
        {
            SingleUuid = Guid.NewGuid()
        });

        originalTask.ContinueWith(result =>
        {
            actualResult = result;
        });

        Assert.That(DiscoveryQueries.Count, Is.EqualTo(1));

        foreach (var (requestId, queryInfo) in DiscoveryQueries)
        {
            EnqueueEvent(OVRPlugin.EventType.SpaceDiscoveryComplete, new OVRDeserialize.SpaceDiscoveryCompleteData
            {
                RequestId = requestId,
                Result = (int)OVRAnchor.FetchResult.Success,
            });
        }

        // Step OVRManager
        yield return null;

        Assert.That(actualResult.HasValue, Is.True);
        var result = actualResult.Value;
        Assert.That(result.Success, Is.True);
        Assert.That(result.Value, Is.SameAs(list));
    }

    static unsafe OVRTelemetryMarker AddAnnotation(OVRTelemetryMarker marker, string key, long[] values)
    {
        fixed (long* ptr = values)
        {
            return marker.AddAnnotation(key, ptr, values?.Length ?? 0);
        }
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator SpaceDiscoveryCapturesExpectedTelemetry()
    {
        var list = new List<OVRAnchor>();
        var task = OVRAnchor.FetchAnchorsAsync(list, new OVRAnchor.FetchOptions
        {
            SingleUuid = Guid.NewGuid(),
            Uuids = Enumerable.Range(0, 10).Select(_ => Guid.NewGuid()).ToArray(),
            SingleComponentType = typeof(OVRRoomLayout),
            ComponentTypes = new[]
            {
                typeof(OVRSemanticLabels),
                typeof(OVRBounded2D),
                typeof(OVRBounded3D),
            }
        });

        var expectedMarker = OVRTelemetry
            .Expect((int)OVRAnchor.Telemetry.MarkerId.DiscoverSpaces)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SynchronousResult, (long)OVRPlugin.Result.Success)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.AsynchronousResult, (long)OVRPlugin.Result.Failure_SpacePermissionInsufficient)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.UuidCount, (long)11)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.TotalFilterCount, (long)(1 + 1 + 4))
            .SetResult(OVRPlugin.Qpl.ResultType.Fail);

        AddAnnotation(expectedMarker, OVRAnchor.Telemetry.Annotation.ComponentTypes, new[]
        {
            (long)OVRPlugin.SpaceComponentType.RoomLayout,
            (long)OVRPlugin.SpaceComponentType.SemanticLabels,
            (long)OVRPlugin.SpaceComponentType.Bounded2D,
            (long)OVRPlugin.SpaceComponentType.Bounded3D,
        });

        foreach (var requestId in DiscoveryQueries.Keys)
        {
            EnqueueEvent(OVRPlugin.EventType.SpaceDiscoveryComplete, new OVRDeserialize.SpaceDiscoveryCompleteData
            {
                RequestId = requestId,
                Result = (int)OVRAnchor.FetchResult.FailurePermissionInsufficient,
            });
        }

        // Step OVRManager
        yield return new WaitWhile(() => task.IsPending);

        OVRTelemetry.TestExpectations();
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator SaveSpacesCapturesExpectedTelemetry()
    {
        var task = OVRAnchor.CreateSpatialAnchorAsync(Pose.identity);
        yield return WaitForTaskCompletion(task, HandleAllSpatialAnchorCreationRequests);
        var anchor = task.GetResult();

        OVRTelemetry
            .Expect((int)OVRAnchor.Telemetry.MarkerId.SaveSpaces)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SynchronousResult, (long)OVRPlugin.Result.Success)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.AsynchronousResult, (long)OVRAnchor.SaveResult.FailureTooDark)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SpaceCount, (long)1)
            .SetResult(OVRPlugin.Qpl.ResultType.Fail);

        anchor.SaveAsync();
        yield return new WaitUntil(() => SaveSpacesRequests.Count > 0);

        foreach (var requestId in SaveSpacesRequests.Keys)
        {
            EnqueueEvent(OVRPlugin.EventType.SpacesSaveResult, new OVRDeserialize.SpacesSaveResultData
            {
                RequestId = requestId,
                Result = OVRAnchor.SaveResult.FailureTooDark
            });
        }

        // Step OVRManager
        yield return new WaitWhile(() => task.IsPending);

        OVRTelemetry.TestExpectations();
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator EraseSpacesCapturesExpectedTelemetry()
    {
        var task = OVRAnchor.CreateSpatialAnchorAsync(Pose.identity);
        yield return WaitForTaskCompletion(task, HandleAllSpatialAnchorCreationRequests);
        var anchor = task.GetResult();

        OVRTelemetry
            .Expect((int)OVRAnchor.Telemetry.MarkerId.EraseSpaces)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SynchronousResult, (long)OVRPlugin.Result.Success)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.AsynchronousResult, (long)OVRAnchor.EraseResult.FailureInsufficientResources)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SpaceCount, (long)1)
            .SetResult(OVRPlugin.Qpl.ResultType.Fail);

        yield return WaitForTaskCompletion(anchor.EraseAsync(), () =>
        {
            foreach (var requestId in EraseSpacesRequests.Keys)
            {
                EnqueueEvent(OVRPlugin.EventType.SpacesEraseResult, new OVRDeserialize.SpacesEraseResultData
                {
                    RequestId = requestId,
                    Result = OVRAnchor.EraseResult.FailureInsufficientResources
                });
            }
        });

        // Step OVRManager
        yield return new WaitWhile(() => task.IsPending);

        OVRTelemetry.TestExpectations();
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator QuerySpacesByUuidCapturesExpectedTelemetry()
    {
        const OVRSpace.StorageLocation storageLocation = OVRSpace.StorageLocation.Cloud;
        const double timeout = Math.PI;
        const int asyncResult = (int)OVRPlugin.Result.Failure_InvalidParameter;

        var task = OVRAnchor.FetchAnchorsAsync(
            Enumerable.Range(0, 10).Select(_ => Guid.NewGuid()), new List<OVRAnchor>(), storageLocation, timeout);

        OVRTelemetry
            .Expect((int)OVRAnchor.Telemetry.MarkerId.QuerySpaces)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SynchronousResult, (long)OVRPlugin.Result.Success)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.AsynchronousResult, (long)asyncResult)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.StorageLocation, (long)storageLocation.ToSpaceStorageLocation())
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.Timeout, (double)timeout)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.UuidCount, (long)10)
            .SetResult(OVRPlugin.Qpl.ResultType.Fail);

        foreach (var requestId in SpaceQueries.Keys)
        {
            EnqueueEvent(OVRPlugin.EventType.SpaceQueryComplete, new OVRDeserialize.SpaceQueryCompleteData
            {
                RequestId = requestId,
                Result = asyncResult,
            });
        }

        // Step OVRManager
        yield return new WaitWhile(() => task.IsPending);

        OVRTelemetry.TestExpectations();
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator QuerySpacesByComponentCapturesExpectedTelemetry()
    {
        const OVRSpace.StorageLocation storageLocation = OVRSpace.StorageLocation.Cloud;
        const double timeout = Math.PI;
        const int asyncResult = (int)OVRPlugin.Result.Failure_InvalidParameter;
        const int maxResults = 123;

        var task = OVRAnchor.FetchAnchorsAsync<OVRRoomLayout>(new List<OVRAnchor>(), storageLocation, maxResults, timeout);

        var expectedMarker = OVRTelemetry
            .Expect((int)OVRAnchor.Telemetry.MarkerId.QuerySpaces)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SynchronousResult, (long)OVRPlugin.Result.Success)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.AsynchronousResult, (long)asyncResult)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.StorageLocation, (long)storageLocation.ToSpaceStorageLocation())
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.Timeout, (double)timeout)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.MaxResults, (long)maxResults)
            .SetResult(OVRPlugin.Qpl.ResultType.Fail);

        AddAnnotation(expectedMarker, OVRAnchor.Telemetry.Annotation.ComponentTypes, new []
        {
            (long)OVRPlugin.SpaceComponentType.RoomLayout
        });

        foreach (var requestId in SpaceQueries.Keys)
        {
            EnqueueEvent(OVRPlugin.EventType.SpaceQueryComplete, new OVRDeserialize.SpaceQueryCompleteData
            {
                RequestId = requestId,
                Result = asyncResult,
            });
        }

        // Step OVRManager
        yield return new WaitWhile(() => task.IsPending);

        OVRTelemetry.TestExpectations();
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator SaveSpaceListCapturesExpectedTelemetry()
    {
        const int asyncResult = (int)OVRPlugin.Result.Success;
        const OVRSpace.StorageLocation storageLocation = OVRSpace.StorageLocation.Cloud;

        var anchor = new GameObject("anchor").AddComponent<OVRSpatialAnchor>();
        yield return null;

        HandleAllSpatialAnchorCreationRequests();
        yield return new WaitWhile(() => anchor.PendingCreation);

        OVRTelemetry
            .Expect((int)OVRAnchor.Telemetry.MarkerId.SaveSpaceList)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SynchronousResult, (long)OVRPlugin.Result.Success)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.AsynchronousResult, (long)asyncResult)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SpaceCount, (long)3)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.StorageLocation, (long)storageLocation.ToSpaceStorageLocation())
            .SetResult(OVRPlugin.Qpl.ResultType.Success);

        var options = new OVRSpatialAnchor.SaveOptions { Storage = storageLocation };
        anchor.SaveAsync(options);
        anchor.SaveAsync(options);
        anchor.SaveAsync(options);

        yield return new WaitUntil(() => SaveSpaceListRequests.Count > 0);

        foreach (var requestId in SaveSpaceListRequests.Keys)
        {
            EnqueueEvent(OVRPlugin.EventType.SpaceListSaveResult, new OVRDeserialize.SpaceListSaveResultData
            {
                RequestId = requestId,
                Result = asyncResult
            });
        }

        // Step OVRManager
        yield return null;

        OVRTelemetry.TestExpectations();
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator EraseSpaceCapturesExpectedTelemetry()
    {
        const int asyncResult = (int)OVRPlugin.Result.Success;
        const OVRSpace.StorageLocation storageLocation = OVRSpace.StorageLocation.Cloud;

        var anchor = new GameObject("anchor").AddComponent<OVRSpatialAnchor>();
        yield return null;

        HandleAllSpatialAnchorCreationRequests();
        yield return new WaitWhile(() => anchor.PendingCreation);

        OVRTelemetry
            .Expect((int)OVRAnchor.Telemetry.MarkerId.EraseSingleSpace)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.SynchronousResult, (long)OVRPlugin.Result.Success)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.AsynchronousResult, (long)asyncResult)
            .AddAnnotation(OVRAnchor.Telemetry.Annotation.StorageLocation, (long)storageLocation.ToSpaceStorageLocation())
            .SetResult(OVRPlugin.Qpl.ResultType.Success);

        anchor.EraseAsync(new OVRSpatialAnchor.EraseOptions { Storage = storageLocation });

        yield return new WaitUntil(() => EraseQueries.Count > 0);

        foreach (var requestId in EraseQueries.Keys)
        {
            EnqueueEvent(OVRPlugin.EventType.SpaceEraseComplete, new OVRDeserialize.SpaceEraseCompleteData
            {
                RequestId = requestId,
                Result = asyncResult
            });
        }

        // Step OVRManager
        yield return null;

        OVRTelemetry.TestExpectations();
    }

    [UnityTest]
    public IEnumerator CanQueryWithZeroFilters()
    {
        OVRResult<List<OVRAnchor>, OVRAnchor.FetchResult>? taskResult = null;
        var list = new List<OVRAnchor>();
        OVRAnchor.FetchAnchorsAsync(list, options: default)
            .ContinueWith(result => taskResult = result);

        foreach (var (requestId, _) in DiscoveryQueries)
        {
            EnqueueEvent(OVRPlugin.EventType.SpaceDiscoveryComplete, new OVRDeserialize.SpaceDiscoveryCompleteData
            {
                RequestId = requestId,
                Result = (int)OVRAnchor.FetchResult.Success,
            });
        }

        // Step OVRManager
        yield return null;

        Assert.That(taskResult.HasValue, Is.True);
        Assert.That(taskResult.Value.Success, Is.True);
    }

    [UnityTest]
    public IEnumerator SpaceDiscoveryIncrementalResultsCallbackIsInvoked()
    {
        OVRPlugin.SpaceDiscoveryResult[] CreateResults(int count)
        {
            return Enumerable.Range(0, count).Select(_ => new OVRPlugin.SpaceDiscoveryResult
            {
                Space = (ulong)Random.Range(0, int.MaxValue),
                Uuid = Guid.NewGuid()
            }).ToArray();
        }

        var resultSets = new[]
        {
            CreateResults(5),
            CreateResults(3),
            CreateResults(1)
        };

        var resultSetIndex = 0;

        OVRResult<List<OVRAnchor>, OVRAnchor.FetchResult>? taskResult = null;

        var list = new List<OVRAnchor>();
        var originalTask = OVRAnchor.FetchAnchorsAsync(list, new OVRAnchor.FetchOptions
        {
            SingleUuid = Guid.NewGuid()
        }, incrementalResultsCallback: (anchors, index) =>
        {
            Assert.That(resultSetIndex, Is.LessThan(resultSets.Length));
            var resultSet = resultSets[resultSetIndex];
            Assert.That(resultSet.Select(r => new OVRAnchor(r.Space, r.Uuid)),
                Is.EqualTo(list.Skip(index).Take(resultSet.Length)));
            resultSetIndex++;
        });
        originalTask.ContinueWith(result =>
        {
            taskResult = result;
        });

        Assert.That(DiscoveryQueries.Count, Is.EqualTo(1));
        var (requestId, _) = DiscoveryQueries.First();

        for (var i = 0; i < resultSets.Length; i++)
        {
            AddSpaceDiscoveryResults(requestId, resultSets[i], isFinal: i == resultSets.Length - 1);
            yield return null;

            // Asserts that the incremental callback was actually invoked.
            Assert.That(resultSetIndex, Is.EqualTo(i + 1));
        }

        Assert.That(originalTask.IsCompleted, Is.True);
        var expectedResults = resultSets
            .SelectMany(r => r.Select(a => new OVRAnchor(a.Space, a.Uuid)));
        Assert.That(list, Is.EqualTo(expectedResults));
    }

    [Test]
    public void FetchOptionsGeneratesCorrectQuery()
    {
        var uuid = Guid.NewGuid();
        var uuids = Enumerable.Range(0, 10).Select(_ => Guid.NewGuid()).ToArray();

        new OVRAnchor.FetchOptions
        {
            SingleUuid = uuid,
            Uuids = uuids,
            SingleComponentType = typeof(OVRBounded2D),
            ComponentTypes = new[] { typeof(OVRBounded3D), typeof(OVRTriangleMesh) },
        }.DiscoverSpaces(out var requestId);

        var query = DiscoveryQueries[requestId];

        var filters = query.Filters.ToHashSet();

        T Find<T>(Func<T, bool> pred)
        {
            foreach (var filter in filters)
            {
                if (filter is T t && pred(t))
                {
                    filters.Remove(filter);
                    return t;
                }
            }

            Assert.Fail("No filter matched the predicate.");
            throw new Exception("Test failed.");
        }

        {
            var filter = Find<DiscoveryFilterInfoIds>(f => f.Ids.Length == 1);
            Assert.That(filter.Ids[0], Is.EqualTo(uuid));
        }

        {
            Find<DiscoveryFilterInfoComponents>(
                f => f.Component == OVRPlugin.SpaceComponentType.Bounded2D);
        }

        {
            Find<DiscoveryFilterInfoComponents>(
                f => f.Component == OVRPlugin.SpaceComponentType.Bounded3D);
        }

        {
            Find<DiscoveryFilterInfoComponents>(f => f.Component == OVRPlugin.SpaceComponentType.TriangleMesh);
        }

        {
            var filter = Find<DiscoveryFilterInfoIds>(f => f.Ids.Length == uuids.Length);
            Assert.That(filter.Ids, Is.EquivalentTo(uuids));
        }

        Assert.That(filters.Count, Is.Zero);
    }
#endif // OVR_INTERNAL_CODE

    [UnityTest]
    public IEnumerator SpatialAnchorCanBeCreatedAndDestroyed()
    {
        var trackingSpacePose = new Pose()
        {
            position = UnityEngine.Random.insideUnitSphere,
            rotation = UnityEngine.Random.rotationUniform
        };

        var task = OVRAnchor.CreateSpatialAnchorAsync(trackingSpacePose);

        yield return WaitForTaskCompletion(task, HandleAllSpatialAnchorCreationRequests);

        var anchor = task.GetResult();
        Assert.AreNotEqual(anchor, OVRAnchor.Null);

        var result = anchor.TryGetComponent(out OVRLocatable locatable);
        Assert.IsTrue(result);

        if (!locatable.IsEnabled)
        {
            yield return WaitForTaskCompletion(locatable.SetEnabledAsync(true), HandleComponentStatusChangeRequest);
        }

        result = locatable.TryGetSpatialAnchorPose(out var pose);
        Assert.IsTrue(result);

        Assert.That(pose.Position, Is.EqualTo(trackingSpacePose.position));
        Assert.That(pose.Rotation, Is.EqualTo(trackingSpacePose.rotation));

        Assert.That(Spaces.ContainsKey(anchor.Handle), Is.True);
        anchor.Dispose();
        Assert.That(Spaces.ContainsKey(anchor.Handle), Is.False);
    }

    private IEnumerator BaseSetup()
    {
        _expectedSpace = CreateSpace(OvrSpaceFBType.plane_anchor);
        _unsupportedSpace = CreateSpace(OvrSpaceFBType.plane_anchor);

        yield return null;
    }

    private IEnumerator WaitForOVRManager()
    {
        while (!OVRManager.OVRManagerinitialized)
        {
            yield return null;
        }
    }

    private IEnumerator WaitForTaskCompletion<T>(OVRTask<T> task, Action handleAction)
    {
        Assert.IsFalse(task.IsCompleted);
        yield return null;

        handleAction?.Invoke();
        yield return null;

        Assert.IsTrue(task.IsCompleted);
    }

    private new void HandleAllSpatialAnchorCreationRequests()
    {
        foreach (var (requestId, request) in SpatialAnchorCreationRequests
                     .Select(kvp => (kvp.Key, kvp.Value)).ToArray())
        {
            var space = new SpatialAnchor();
            var locatable = space.AddComponent<Locatable>();
            locatable.SpaceLocation.pose = request.PoseInSpace;
            HandleSpatialAnchorCreationRequest(new OVRDeserialize.SpatialAnchorCreateCompleteData
            {
                RequestId = requestId,
                Result = (int)OVRPlugin.Result.Success,
                Space = space,
                Uuid = space.Uuid
            }, space);
        }
    }

    private void HandleComponentStatusChangeRequest() => HandleComponentStatusChangeRequest(OVRPlugin.Result.Success);

    private void HandleComponentStatusChangeRequest(OVRPlugin.Result result)
    {
        foreach (var (requestId, request) in ComponentStatusChangeRequests
                     .Select(kvp => (kvp.Key, kvp.Value)).ToArray())
        {
            Assert.True(Spaces.TryGetValue(request.Space, out var space), "the space should be found");
            Assert.True(space is SpatialAnchor, "the space should be of type SpatialAnchor");

            var spatialAnchor = (SpatialAnchor)space;

            var response = new OVRDeserialize.SpaceSetComponentStatusCompleteData
            {
                ComponentType = request.ComponentType,
                Enabled = (int)request.Enable,
                RequestId = requestId,
                Result = (int)result,
                Space = spatialAnchor,
                Uuid = spatialAnchor.Uuid
            };

            HandleComponentStatusChangeRequest(response);
        }
    }

    private class ComponentTestHelper<TMockType, TOvrType>
        where TMockType : class, IComponent
        where TOvrType : struct, IOVRAnchorComponent<TOvrType>
    {
        public delegate void SetupSceneDelegate(out TMockType mockComponent);

        public delegate IEnumerator TestFeaturesDelegate(TMockType mockComponent, TOvrType component);

        private readonly SetupSceneDelegate _setupDelegate;
        private readonly TestFeaturesDelegate _testFeaturesDelegate;
        private readonly OVRAnchorPlayModeTests _fixture;

        private TMockType _mockComponent;
        private TOvrType _ovrComponent;

        public ComponentTestHelper(OVRAnchorPlayModeTests fixture, SetupSceneDelegate setupDelegate,
            TestFeaturesDelegate testFeaturesDelegate)
        {
            _fixture = fixture;
            _setupDelegate = setupDelegate;
            _testFeaturesDelegate = testFeaturesDelegate;
        }

        public IEnumerator TestComponentInternal()
        {
            _setupDelegate?.Invoke(out _mockComponent);

            var list = OVRObjectPool.Get<List<OVRAnchor>>();
            var task = OVRAnchor.FetchAnchorsAsync(
                new Guid[] { _fixture._expectedSpace.Uuid, _fixture._unsupportedSpace.Uuid }, list);
            yield return _fixture.WaitForTaskCompletion(task, _fixture.HandleAllSpaceQueries);
            var result = task.GetResult();
            Assert.IsTrue(result);
            foreach (var anchor in list)
            {
                yield return TestComponent(anchor);
            }

            OVRObjectPool.Return(list);
        }

        private IEnumerator TestComponent(OVRAnchor anchor)
        {
            if (anchor == OVRAnchor.Null)
            {
                yield break;
            }

            var supportedComponents = new List<OVRPlugin.SpaceComponentType>();
            Assert.IsTrue(anchor.GetSupportedComponents(supportedComponents));

            var supportsComponent = anchor.SupportsComponent<TOvrType>();
            if (!supportsComponent)
            {
                Assert.Throws<InvalidOperationException>(() =>
                {
                    var invalidComponent = anchor.GetComponent<TOvrType>();
                });
                yield break;
            }

            var component = anchor.GetComponent<TOvrType>();
            Assert.IsFalse(component.IsNull);
            Assert.AreEqual(component.Handle, anchor.Handle);

            if (!component.IsEnabled)
            {
                var task = component.SetEnabledAsync(true);

                yield return _fixture.WaitForTaskCompletion(task, _fixture.HandleAllSpaceQueries);

                var result = task.GetResult();
                Assert.IsTrue(result);
                Assert.IsTrue(component.IsEnabled);
            }

            yield return _testFeaturesDelegate?.Invoke(_mockComponent, component);
        }
    }

    private IEnumerator TestComponent<TMockType, TOvrType>(
        OVRAnchorPlayModeTests fixture,
        ComponentTestHelper<TMockType, TOvrType>.SetupSceneDelegate setupScene,
        ComponentTestHelper<TMockType, TOvrType>.TestFeaturesDelegate testFeatures)
        where TMockType : class, IComponent
        where TOvrType : struct, IOVRAnchorComponent<TOvrType>
    {
        var helper = new ComponentTestHelper<TMockType, TOvrType>(this, setupScene, testFeatures);
        yield return helper.TestComponentInternal();
    }

    [UnityTest]
    public IEnumerator TestOVRSemanticLabels()
    {
        yield return TestComponent<SemanticLabels, OVRSemanticLabels>(this, SetupOVRSemanticLabels,
            TestsOVRSemanticLabelsFeatures);
    }

    private void SetupOVRSemanticLabels(out SemanticLabels mockComponent)
    {
        mockComponent = _expectedSpace.AddComponent<SemanticLabels>();
        mockComponent.Labels = new[]
        {
            OVRSceneManager.Classification.Table,
            OVRSceneManager.Classification.Other,
            OVRSceneManager.Classification.InvisibleWallFace,
        };
        _unsupportedSpace.AddUnsupportedComponent<SemanticLabels>();
    }

    private IEnumerator TestsOVRSemanticLabelsFeatures(SemanticLabels mockComponent, OVRSemanticLabels ovrComponent)
    {
        Assert.That(ovrComponent.Labels, Is.EqualTo(mockComponent.ToString()));
        yield return null;
    }

    [UnityTest]
    public IEnumerator TestOVRBounded2D()
    {
        yield return TestComponent<Bounded2D, OVRBounded2D>(this, SetupOVRBounded2D, TestOVRBounded2DFeatures);
    }

    private void SetupOVRBounded2D(out Bounded2D mockComponent)
    {
        mockComponent = _expectedSpace.AddComponent<Bounded2D>();
        mockComponent.BoundingBox = new OVRPlugin.Rectf()
        {
            Pos = new OVRPlugin.Vector2f() { x = 1.0f, y = 2.0f },
            Size = new OVRPlugin.Sizef() { w = 3.0f, h = 4.0f }
        };
        const int boundaryCount = 4;
        mockComponent.Boundary = new Vector2[boundaryCount];
        for (var i = 0; i < boundaryCount; i++)
        {
            mockComponent.Boundary[i] = UnityEngine.Random.insideUnitCircle * 10.0f;
        }

        _unsupportedSpace.AddUnsupportedComponent<Bounded2D>();
    }

    private IEnumerator TestOVRBounded2DFeatures(Bounded2D mockComponent, OVRBounded2D ovrComponent)
    {
        // Left-handed, z facing normal, 2D rect
        var rect = ovrComponent.BoundingBox;
        var mockRect = mockComponent.BoundingBox;
        Assert.That(rect.x, Is.EqualTo(-mockRect.Pos.x - mockRect.Size.w)); // Flipped
        Assert.That(rect.y, Is.EqualTo(mockRect.Pos.y));
        Assert.That(rect.width, Is.EqualTo(mockRect.Size.w));
        Assert.That(rect.height, Is.EqualTo(mockRect.Size.h));
        var result = ovrComponent.TryGetBoundaryPointsCount(out var boundaryPointsCount);
        Assert.IsTrue(result);
        Assert.That(boundaryPointsCount, Is.EqualTo(mockComponent.Boundary.Length));
        var positions = new NativeArray<Vector2>(boundaryPointsCount, Allocator.Temp);
        result = ovrComponent.TryGetBoundaryPoints(positions);
        Assert.IsTrue(result);
        for (var i = 0; i < positions.Length; i++)
        {
            var reversedI = positions.Length - 1 - i;
            Assert.That(positions[i].x, Is.EqualTo(-mockComponent.Boundary[reversedI].x));
            Assert.That(positions[i].y, Is.EqualTo(mockComponent.Boundary[reversedI].y));
        }

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestOVRBounded3D()
    {
        yield return TestComponent<Bounded3D, OVRBounded3D>(this, SetupOVRBounded3D, TestOVRBounded3DFeatures);
    }

    private void SetupOVRBounded3D(out Bounded3D mockComponent)
    {
        mockComponent = _expectedSpace.AddComponent<Bounded3D>();
        mockComponent.BoundingBox = new OVRPlugin.Boundsf()
        {
            Pos = new OVRPlugin.Vector3f() { x = 1.0f, y = 2.0f, z = 3.0f },
            Size = new OVRPlugin.Size3f() { w = 1.0f, h = 2.0f, d = 3.0f }
        };
        _unsupportedSpace.AddUnsupportedComponent<Bounded3D>();
    }

    private IEnumerator TestOVRBounded3DFeatures(Bounded3D mockComponent, OVRBounded3D ovrComponent)
    {
        var bounds = ovrComponent.BoundingBox;
        var mockBounds = mockComponent.BoundingBox;
        Assert.That(bounds.center.x, Is.EqualTo(-mockBounds.Pos.x - mockBounds.Size.w + mockBounds.Size.w * 0.5f));
        Assert.That(bounds.center.y, Is.EqualTo(mockBounds.Pos.y + mockBounds.Size.h * 0.5f));
        Assert.That(bounds.center.z, Is.EqualTo(mockBounds.Pos.z + mockBounds.Size.d * 0.5f));
        Assert.That(bounds.extents.x, Is.EqualTo(mockBounds.Size.w * 0.5f));
        Assert.That(bounds.extents.y, Is.EqualTo(mockBounds.Size.h * 0.5f));
        Assert.That(bounds.extents.z, Is.EqualTo(mockBounds.Size.d * 0.5f));
        yield return null;
    }

    [UnityTest]
    public IEnumerator TestOVRLocatable()
    {
        yield return TestComponent<Locatable, OVRLocatable>(this, SetupOVRLocatable, TestOVRLocatableFeatures);
    }

    private void SetupOVRLocatable(out Locatable mockComponent)
    {
        mockComponent = _expectedSpace.AddComponent<Locatable>();
        mockComponent.SpaceLocation = new OVRPlugin.SpaceLocationf()
        {
            pose = new OVRPlugin.Posef()
            {
                Orientation = new OVRPlugin.Quatf()
                {
                    x = 0.2692556f, y = 0.3862204f, z = 0.1852638f, w = 0.8625616f
                },
                Position = new OVRPlugin.Vector3f()
                {
                    x = 1.0f, y = 2.0f, z = 3.0f
                }
            },
            locationFlags = OVRPlugin.SpaceLocationFlags.PositionValid | OVRPlugin.SpaceLocationFlags.PositionTracked |
                            OVRPlugin.SpaceLocationFlags.OrientationValid |
                            OVRPlugin.SpaceLocationFlags.OrientationTracked
        };
        _unsupportedSpace.AddUnsupportedComponent<Locatable>();
    }

    private static readonly Quaternion RotateY180 = new Quaternion(0, 1, 0, 0);

    private IEnumerator TestOVRLocatableFeatures(Locatable mockComponent, OVRLocatable ovrComponent)
    {
        var mockPose = new OVRPose
        {
            position = mockComponent.SpaceLocation.pose.Position.FromFlippedZVector3f(),
            orientation = mockComponent.SpaceLocation.pose.Orientation.FromFlippedZQuatf() * RotateY180
        };
        TestGetTrackingSpacePose(ovrComponent.TryGetSceneAnchorPose(out var trackingSpacePose), mockPose,
            trackingSpacePose);
        mockPose = new OVRPose
        {
            position = mockComponent.SpaceLocation.pose.Position.FromFlippedZVector3f(),
            orientation = mockComponent.SpaceLocation.pose.Orientation.FromFlippedZQuatf()
        };
        TestGetTrackingSpacePose(ovrComponent.TryGetSpatialAnchorPose(out trackingSpacePose), mockPose,
            trackingSpacePose);
        yield return null;
    }

    private void TestGetTrackingSpacePose(bool result, OVRPose mockPose, OVRLocatable.TrackingSpacePose pose)
    {
        Assert.IsTrue(result);
        var mockPosition = mockPose.position;
        Assert.IsTrue(pose.Position == mockPosition);
        var mockRotation = mockPose.orientation;
        Assert.IsTrue(pose.Rotation == mockRotation);

        mockPose = mockPose.ToWorldSpacePose(_ovrCamera);
        Assert.IsTrue(result);
        var worldPosition = pose.ComputeWorldPosition(_ovrCamera);
        mockPosition = mockPose.position;
        Assert.IsTrue(worldPosition == mockPosition);
        var worldRotation = pose.ComputeWorldRotation(_ovrCamera);
        mockRotation = mockPose.orientation;
        Assert.IsTrue(worldRotation == mockRotation);
    }

    [UnityTest]
    public IEnumerator TestOVRRoomLayout()
    {
        yield return TestComponent<RoomLayout, OVRRoomLayout>(this, SetupOVRRoomLayout, AdditionalTestOVRRoomLayout);
    }

    private void SetupOVRRoomLayout(out RoomLayout mockComponent)
    {
        var ceiling = CreateSpace(OvrSpaceFBType.plane_anchor);
        var floor = CreateSpace(OvrSpaceFBType.plane_anchor);
        var wall = CreateSpace(OvrSpaceFBType.plane_anchor);
        mockComponent = _expectedSpace.AddComponent<RoomLayout>();
        mockComponent.Ceiling = ceiling;
        mockComponent.Floor = floor;
        mockComponent.Walls = new[] { wall };
        _unsupportedSpace.AddUnsupportedComponent<RoomLayout>();
    }

    private IEnumerator AdditionalTestOVRRoomLayout(RoomLayout mockComponent, OVRRoomLayout ovrComponent)
    {
        ovrComponent.TryGetRoomLayout(out var ceiling, out var floor, out var walls);
        Assert.That(ceiling, Is.EqualTo(mockComponent.Ceiling.Uuid));
        Assert.That(floor, Is.EqualTo(mockComponent.Floor.Uuid));
        Assert.That(walls.Length, Is.EqualTo(mockComponent.Walls.Length));
        Assert.That(walls[0], Is.EqualTo(mockComponent.Walls[0].Uuid));
        var anchors = OVRObjectPool.List<OVRAnchor>();
        var task = ovrComponent.FetchLayoutAnchorsAsync(anchors);
        yield return WaitForTaskCompletion(task, HandleAllSpaceQueries);
        var result = task.GetResult();
        Assert.IsTrue(result);
        Assert.That(anchors.Count, Is.EqualTo(3));
        foreach (var anchor in anchors)
        {
            Assert.IsTrue(mockComponent.Uuids.Contains(anchor.Uuid));
        }

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestOVRAnchorContainer()
    {
        yield return TestComponent<SpaceContainer, OVRAnchorContainer>(this, SetupOVRAnchorContainer,
            TestOVRAnchorContainerFeatures);
    }

    private void SetupOVRAnchorContainer(out SpaceContainer mockComponent)
    {
        mockComponent = _expectedSpace.AddComponent<SpaceContainer>();
        var extra = CreateSpace(OvrSpaceFBType.plane_anchor);
        var spaces = new HashSet<Space> { extra };
        SpaceContainers[_expectedSpace] = spaces;
        _unsupportedSpace.AddUnsupportedComponent<SpaceContainer>();
    }

    private IEnumerator TestOVRAnchorContainerFeatures(SpaceContainer mockComponent, OVRAnchorContainer component)
    {
        var anchors = OVRObjectPool.Get<List<OVRAnchor>>();
        var task = component.FetchChildrenAsync(anchors);
        yield return WaitForTaskCompletion(task, HandleAllSpaceQueries);
        var result = task.GetResult();
        Assert.IsTrue(result);
        var mockAnchors = SpaceContainers[_expectedSpace];
        Assert.That(anchors.Count, Is.EqualTo(mockAnchors.Count));
        var i = 0;
        foreach (var mockAnchor in mockAnchors)
        {
            Assert.That(mockAnchor.Uuid, Is.EqualTo(anchors[i].Uuid));
            i++;
        }

        yield return null;
    }

#if OVR_INTERNAL_CODE // Room entity info
    [Test]
    public void RoomLabelCanBeRetrieved()
    {
        // OpenXR uses UTF8, so non-ASCII characters should be supported
        const string expectedValue = "Bedroom (寝室)";

        var room = CreateSpace(OvrSpaceFBType.room_entity);
        room.AddComponent<RoomLabel>().Value = expectedValue;
        var anchor = new OVRAnchor(room.Handle, room.Uuid);

        Assert.That(anchor.TryGetComponent<OVRRoomLabel>(out var roomLabelComponent), Is.True);
        Assert.That(roomLabelComponent.IsEnabled, Is.True);
        Assert.That(roomLabelComponent.TryGetValue(out var actualValue), Is.True);
        Assert.That(actualValue, Is.EqualTo(expectedValue));
    }

    [Test]
    public void RoomLabelCanBeRetrievedViaPropertyAccessor()
    {
        const string expectedValue = "some value";

        var room = CreateSpace(OvrSpaceFBType.room_entity);
        room.AddComponent<RoomLabel>().Value = expectedValue;
        var anchor = new OVRAnchor(room.Handle, room.Uuid);

        Assert.That(anchor.TryGetComponent<OVRRoomLabel>(out var roomLabel), Is.True);
        Assert.That(roomLabel.Value, Is.EqualTo(expectedValue));
    }

    [Test]
    public void TryGetValueFailsIfRoomLabelComponentIsDisabled()
    {
        var room = CreateSpace(OvrSpaceFBType.room_entity);
        var anchor = new OVRAnchor(room.Handle, room.Uuid);
        Assert.That(anchor.TryGetComponent<OVRRoomLabel>(out var roomLabel), Is.True);
        Assert.That(roomLabel.IsEnabled, Is.False);
        Assert.That(roomLabel.TryGetValue(out _), Is.False);
    }

    [Test]
    public void RoomLabelAccessorThrowsIfComponentIsDisabled()
    {
        var room = CreateSpace(OvrSpaceFBType.room_entity);
        var anchor = new OVRAnchor(room.Handle, room.Uuid);
        Assert.That(anchor.TryGetComponent<OVRRoomLabel>(out var roomLabel), Is.True);
        Assert.Throws<InvalidOperationException>(() => _ = roomLabel.Value);
    }
#endif

#if OVR_INTERNAL_CODE // XR_META_return_to_room
    [Test]
    public void RequestReturnToRoomInvokedWithCorrectGuids()
    {
        var guids = Enumerable.Range(0, 10).Select(_ => Guid.NewGuid()).ToArray();
        Assert.That(OVRAnchor.RequestReturnToRoom(guids).Success, Is.True);
        Assert.That(ReturnToRoomUuids, Is.EquivalentTo(guids));
    }

    [Test]
    public void RequestReturnToRoomCanBeInvokedWithNull()
    {
        Assert.That(OVRAnchor.RequestReturnToRoom(null).Success, Is.True);
        Assert.That(ReturnToRoomUuids.Length, Is.EqualTo(0));
    }
#endif

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator SpaceComponentChangesCanBeDeferred()
    {
        var creationTask = OVRAnchor.CreateSpatialAnchorAsync(Pose.identity);
        yield return WaitForTaskCompletion(creationTask, HandleAllSpatialAnchorCreationRequests);
        var anchor = creationTask.GetResult();

        // Make sure it doesn't have the locatable component
        foreach (var space in Spaces.Values)
        {
            space.RemoveComponent<Locatable>();
        }

        var locatable = anchor.GetComponent<OVRLocatable>();

        var tasks = new OVRTask<bool>[10];

        tasks[0] = locatable.SetEnabledAsync(true);
        Assert.That(tasks[0].IsPending);

        for (var i = 1; i < tasks.Length; i++)
        {
            tasks[i] = locatable.SetEnabledAsync(true);
            Assert.That(tasks[i].IsPending);
        }

        yield return WaitForTaskCompletion(tasks[0], HandleComponentStatusChangeRequest);

        foreach (var task in tasks)
        {
            Assert.That(task.GetResult(), Is.True);
        }
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator SpaceComponentChangesCanBeQueued()
    {
        var creationTask = OVRAnchor.CreateSpatialAnchorAsync(Pose.identity);
        yield return WaitForTaskCompletion(creationTask, HandleAllSpatialAnchorCreationRequests);
        var anchor = creationTask.GetResult();

        foreach (var space in Spaces.Values)
        {
            space.RemoveComponent<Locatable>();
        }

        var locatable = anchor.GetComponent<OVRLocatable>();
        var setEnabledTask = locatable.SetEnabledAsync(true);
        var setDisabledTask = locatable.SetEnabledAsync(false); // this should defer the change until after task1 is finished

        yield return WaitForTaskCompletion(setEnabledTask, HandleComponentStatusChangeRequest);
        Assert.That(setDisabledTask.IsPending);

        // There should be a new request for the disabled task
        Assert.That(ComponentStatusChangeRequests.Count, Is.EqualTo(1));

        yield return WaitForTaskCompletion(setDisabledTask, HandleComponentStatusChangeRequest);
        Assert.That(setDisabledTask.GetResult(), Is.EqualTo(true));
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator DeferredSpaceComponentStatusChangeRequestsHandleFailure()
    {
        var creationTask = OVRAnchor.CreateSpatialAnchorAsync(Pose.identity);
        yield return WaitForTaskCompletion(creationTask, HandleAllSpatialAnchorCreationRequests);
        var anchor = creationTask.GetResult();

        foreach (var space in Spaces.Values)
        {
            space.RemoveComponent<Locatable>();
        }

        var locatable = anchor.GetComponent<OVRLocatable>();
        var setEnabledTask = locatable.SetEnabledAsync(true);
        var setDisabledTask = locatable.SetEnabledAsync(false);

        // First task to enable should complete successfully
        yield return WaitForTaskCompletion(setEnabledTask, HandleComponentStatusChangeRequest);

        Assert.That(setDisabledTask.IsPending);

        // This time, we service the set-disabled request, but we test the case where that request fails
        yield return WaitForTaskCompletion(setDisabledTask, () =>
            HandleComponentStatusChangeRequest(OVRPlugin.Result.Failure));

        // The task should have completed in a failed state
        Assert.That(setDisabledTask.GetResult(), Is.False);
    }

    [UnityTest, Timeout(1500)]
    public IEnumerator DeferredSpaceComponentStatusChangeRequestFailsWhenTimeoutExceeded()
    {
        var creationTask = OVRAnchor.CreateSpatialAnchorAsync(Pose.identity);
        yield return WaitForTaskCompletion(creationTask, HandleAllSpatialAnchorCreationRequests);
        var anchor = creationTask.GetResult();

        var locatable = anchor.GetComponent<OVRLocatable>();
        var setDisabledTask = locatable.SetEnabledAsync(false);
        var setEnabledTask = locatable.SetEnabledAsync(true, timeout: .5);

        // Pretend that disabling locatable takes a full second
        yield return new WaitForSeconds(1);
        yield return WaitForTaskCompletion(setDisabledTask, HandleComponentStatusChangeRequest);

        // The task setEnabledTask should fail and there should be no new pending change requests
        Assert.That(ComponentStatusChangeRequests.Count, Is.Zero);
        Assert.That(setEnabledTask.GetResult(), Is.False);
    }

    [UnityTest, Timeout(1500)]
    public IEnumerator DeferredSpaceComponentStatusChangeRequestTimeoutsAreAdjustedByDelayedStartTime()
    {
        var creationTask = OVRAnchor.CreateSpatialAnchorAsync(Pose.identity);
        yield return WaitForTaskCompletion(creationTask, HandleAllSpatialAnchorCreationRequests);
        var anchor = creationTask.GetResult();

        var locatable = anchor.GetComponent<OVRLocatable>();
        var setDisabledTask = locatable.SetEnabledAsync(false);
        var setEnabledTask = locatable.SetEnabledAsync(true, timeout: 1);

        // Pretend that disabling locatable takes half a second
        yield return new WaitForSeconds(0.5f);
        yield return WaitForTaskCompletion(setDisabledTask, HandleComponentStatusChangeRequest);

        // The task setEnabledTask should still succeed because we are within the allowed timeout

        Assert.That(ComponentStatusChangeRequests.Count, Is.EqualTo(1));
        Assert.That(ComponentStatusChangeRequests.First().Value.Timeout, Is.GreaterThan(0));
        yield return WaitForTaskCompletion(setEnabledTask, HandleComponentStatusChangeRequest);

        Assert.That(setEnabledTask.GetResult(), Is.True);
    }

}

#endif // OVRPLUGIN_TESTING
