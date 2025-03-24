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
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using static OVRPlugin;
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
using TaskResult = OVRResult<System.Collections.Generic.List<OVRAnchor>, OVRAnchor.FetchResult>;
using IncrementalResultsDelegate = System.Action<System.Collections.Generic.List<OVRAnchor>, int>;
#endif

/// <summary>
/// Represents an anchor.
/// </summary>
/// <remarks>
/// Scenes anchors are uniquely identified with their <see cref="Uuid"/>.
/// <para>You may dispose of an anchor by calling their <see cref="Dispose"/> method.</para>
/// </remarks>
public readonly partial struct OVRAnchor : IEquatable<OVRAnchor>, IDisposable
{
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
    [OVRResultStatus]
    public enum SaveResult
    {
        /// <summary>
        /// The operation succeeded.
        /// </summary>
        Success = Result.Success,

        /// <summary>
        /// The operation failed.
        /// </summary>
        Failure = Result.Failure,

        /// <summary>
        /// At least one anchor is invalid.
        /// </summary>
        FailureInvalidAnchor = Result.Failure_HandleInvalid,

        /// <summary>
        /// Invalid data.
        /// </summary>
        FailureDataIsInvalid = Result.Failure_DataIsInvalid,

        /// <summary>
        /// Resource limitation prevented this operation from executing.
        /// </summary>
        /// <remarks>
        ///  Recommend retrying, perhaps after a short delay and/or reducing memory consumption.
        /// </remarks>
        FailureInsufficientResources = Result.Failure_SpaceInsufficientResources,

        /// <summary>
        /// Operation could not be completed until resources used are reduced or storage expanded.
        /// </summary>
        FailureStorageAtCapacity = Result.Failure_SpaceStorageAtCapacity,

        /// <summary>
        /// Insufficient view.
        /// </summary>
        /// <remarks>
        /// The user needs to look around the environment more for anchor tracking to function.
        /// </remarks>
        FailureInsufficientView = Result.Failure_SpaceInsufficientView,

        /// <summary>
        /// Insufficient permission.
        /// </summary>
        /// <remarks>
        /// Recommend confirming the status of the required permissions needed for using anchor APIs.
        /// </remarks>
        FailurePermissionInsufficient = Result.Failure_SpacePermissionInsufficient,

        /// <summary>
        /// Operation canceled due to rate limiting.
        /// </summary>
        /// <remarks>
        /// Recommend retrying after a short delay.
        /// </remarks>
        FailureRateLimited = Result.Failure_SpaceRateLimited,

        /// <summary>
        /// Too dark.
        /// </summary>
        /// <remarks>
        /// The environment is too dark to save the anchor.
        /// </remarks>
        FailureTooDark = Result.Failure_SpaceTooDark,

        /// <summary>
        /// Too bright.
        /// </summary>
        /// <remarks>
        /// The environment is too bright to save the anchor.
        /// </remarks>
        FailureTooBright = Result.Failure_SpaceTooBright,
    }

    [OVRResultStatus]
    public enum EraseResult
    {
        /// <summary>
        /// The operation succeeded.
        /// </summary>
        Success = Result.Success,

        /// <summary>
        /// The operation failed.
        /// </summary>
        Failure = Result.Failure,

        /// <summary>
        /// At least one anchor is invalid.
        /// </summary>
        FailureInvalidAnchor = Result.Failure_HandleInvalid,

        /// <summary>
        /// Invalid data.
        /// </summary>
        FailureDataIsInvalid = Result.Failure_DataIsInvalid,

        /// <summary>
        /// Resource limitation prevented this operation from executing.
        /// </summary>
        /// <remarks>
        ///  Recommend retrying, perhaps after a short delay and/or reducing memory consumption.
        /// </remarks>
        FailureInsufficientResources = Result.Failure_SpaceInsufficientResources,

        /// <summary>
        /// Insufficient permission.
        /// </summary>
        /// <remarks>
        /// Recommend confirming the status of the required permissions needed for using anchor APIs.
        /// </remarks>
        FailurePermissionInsufficient = Result.Failure_SpacePermissionInsufficient,

        /// <summary>
        /// Operation canceled due to rate limiting.
        /// </summary>
        /// <remarks>
        /// Recommend retrying after a short delay.
        /// </remarks>
        FailureRateLimited = Result.Failure_SpaceRateLimited,
    }

    [OVRResultStatus]
    public enum FetchResult
    {
        /// <summary>
        /// The operation succeeded.
        /// </summary>
        Success = Result.Success,

        /// <summary>
        /// The operation failed.
        /// </summary>
        Failure = Result.Failure,

        /// <summary>
        /// Invalid data.
        /// </summary>
        FailureDataIsInvalid = Result.Failure_DataIsInvalid,

        /// <summary>
        /// One of the <see cref="FetchOptions"/> was invalid.
        /// </summary>
        /// <remarks>
        /// This can happen, for example, if you query for an invalid component type.
        /// </remarks>
        FailureInvalidOption = Result.Failure_InvalidParameter,

        /// <summary>
        /// Resource limitation prevented this operation from executing.
        /// </summary>
        /// <remarks>
        ///  Recommend retrying, perhaps after a short delay and/or reducing memory consumption.
        /// </remarks>
        FailureInsufficientResources = Result.Failure_SpaceInsufficientResources,

        /// <summary>
        /// Insufficient view.
        /// </summary>
        /// <remarks>
        /// The user needs to look around the environment more for anchor tracking to function.
        /// </remarks>
        FailureInsufficientView = Result.Failure_SpaceInsufficientView,

        /// <summary>
        /// Insufficient permission.
        /// </summary>
        /// <remarks>
        /// Recommend confirming the status of the required permissions needed for using anchor APIs.
        /// </remarks>
        FailurePermissionInsufficient = Result.Failure_SpacePermissionInsufficient,

        /// <summary>
        /// Operation canceled due to rate limiting.
        /// </summary>
        /// <remarks>
        /// Recommend retrying after a short delay.
        /// </remarks>
        FailureRateLimited = Result.Failure_SpaceRateLimited,

        /// <summary>
        /// Too dark.
        /// </summary>
        /// <remarks>
        /// The environment is too dark to save the anchor.
        /// </remarks>
        FailureTooDark = Result.Failure_SpaceTooDark,

        /// <summary>
        /// Too bright.
        /// </summary>
        /// <remarks>
        /// The environment is too bright to save the anchor.
        /// </remarks>
        FailureTooBright = Result.Failure_SpaceTooBright,
    }
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

    #region Static

    public static readonly OVRAnchor Null = new OVRAnchor(0, Guid.Empty);

    internal static unsafe Result SaveSpaceList(ulong* spaces, uint numSpaces, SpaceStorageLocation location,
        out ulong requestId)
    {
        var marker = OVRTelemetry
            .Start((int)Telemetry.MarkerId.SaveSpaceList)
            .AddAnnotation(Telemetry.Annotation.SpaceCount, (long)numSpaces)
            .AddAnnotation(Telemetry.Annotation.StorageLocation, (long)location);

        var result = OVRPlugin.SaveSpaceList(spaces, numSpaces, location, out requestId);

        Telemetry.SetSyncResult(marker, requestId, result);
        return result;
    }

    // Invoked by OVRManager event loop
    internal static void OnSpaceListSaveResult(OVRDeserialize.SpaceListSaveResultData eventData)
        => Telemetry.SetAsyncResultAndSend(Telemetry.MarkerId.SaveSpaceList, eventData.RequestId, eventData.Result);

    internal static Result EraseSpace(ulong space, SpaceStorageLocation location, out ulong requestId)
    {
        var marker = OVRTelemetry
            .Start((int)Telemetry.MarkerId.EraseSingleSpace)
            .AddAnnotation(Telemetry.Annotation.StorageLocation, (long)location);

        var result = OVRPlugin.EraseSpaceWithResult(space, location, out requestId);

        Telemetry.SetSyncResult(marker, requestId, result);
        return result;
    }

    // Invoked by OVRManager event loop
    internal static void OnSpaceEraseComplete(OVRDeserialize.SpaceEraseCompleteData eventData)
        => Telemetry.SetAsyncResultAndSend(Telemetry.MarkerId.EraseSingleSpace, eventData.RequestId, eventData.Result);

    internal static OVRPlugin.SpaceQueryInfo GetQueryInfo(SpaceComponentType type,
        OVRSpace.StorageLocation location, int maxResults, double timeout) => new OVRSpaceQuery.Options
        {
            QueryType = OVRPlugin.SpaceQueryType.Action,
            ActionType = OVRPlugin.SpaceQueryActionType.Load,
            ComponentFilter = type,
            Location = location,
            Timeout = timeout,
            MaxResults = maxResults,
        }.ToQueryInfo();

    internal static OVRPlugin.SpaceQueryInfo GetQueryInfo(IEnumerable<Guid> uuids,
        OVRSpace.StorageLocation location, double timeout) => new OVRSpaceQuery.Options
        {
            QueryType = OVRPlugin.SpaceQueryType.Action,
            ActionType = OVRPlugin.SpaceQueryActionType.Load,
            UuidFilter = uuids,
            Location = location,
            Timeout = timeout,
            MaxResults = OVRSpaceQuery.Options.MaxUuidCount,
        }.ToQueryInfo();

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE

    internal static OVRPlugin.SpaceQueryInfo2 GetQueryInfoFromLocalGroup(Guid localGroupUuid,
        OVRSpace.StorageLocation location, double timeout) => new OVRSpaceQuery.Options
    {
        QueryType = OVRPlugin.SpaceQueryType.Action,
        ActionType = OVRPlugin.SpaceQueryActionType.Load,
        LocalGroupUuidFilter = localGroupUuid,
        Location = location,
        Timeout = timeout,
        MaxResults = OVRSpaceQuery.Options.MaxUuidCount,
    }.ToQueryInfo2();

#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

    internal static OVRTask<bool> FetchAnchorsAsync(SpaceComponentType type, IList<OVRAnchor> anchors,
        OVRSpace.StorageLocation location = OVRSpace.StorageLocation.Local,
        int maxResults = OVRSpaceQuery.Options.MaxUuidCount, double timeout = 0.0)
        => FetchAnchors(anchors, GetQueryInfo(type, location, maxResults, timeout));

    /// <summary>
    /// Asynchronous method that fetches anchors with a specific component.
    /// </summary>
    /// <typeparam name="T">The type of component the fetched anchor must have.</typeparam>
    /// <param name="anchors">IList that will get cleared and populated with the requested anchors.</param>s
    /// <param name="location">Storage location to query</param>
    /// <param name="maxResults">The maximum number of results the query can return</param>
    /// <param name="timeout">Timeout in seconds for the query.</param>
    /// <remarks>Dispose of the returned <see cref="OVRTask{T}"/> if you don't use the results</remarks>
    /// <returns>An <see cref="OVRTask{T}"/> that will eventually let you test if the fetch was successful or not.
    /// If the result is true, then the <see cref="anchors"/> parameter has been populated with the requested anchors.</returns>
    /// <exception cref="System.ArgumentNullException">Thrown if <paramref name="anchors"/> is `null`.</exception>
    public static OVRTask<bool> FetchAnchorsAsync<T>(IList<OVRAnchor> anchors,
        OVRSpace.StorageLocation location = OVRSpace.StorageLocation.Local,
        int maxResults = OVRSpaceQuery.Options.MaxUuidCount, double timeout = 0.0)
        where T : struct, IOVRAnchorComponent<T>
    {
        if (anchors == null)
        {
            throw new ArgumentNullException(nameof(anchors));
        }

        return FetchAnchorsAsync(default(T).Type, anchors, location, maxResults, timeout);
    }

    /// <summary>
    /// Asynchronous method that fetches anchors with specifics uuids.
    /// </summary>
    /// <param name="uuids">Enumerable of uuids that anchors fetched must verify</param>
    /// <param name="anchors">IList that will get cleared and populated with the requested anchors.</param>s
    /// <param name="location">Storage location to query</param>
    /// <param name="timeout">Timeout in seconds for the query.</param>
    /// <remarks>Dispose of the returned <see cref="OVRTask{T}"/> if you don't use the results</remarks>
    /// <returns>An <see cref="OVRTask{T}"/> that will eventually let you test if the fetch was successful or not.
    /// If the result is true, then the <see cref="anchors"/> parameter has been populated with the requested anchors.</returns>
    /// <exception cref="System.ArgumentNullException">Thrown if <paramref name="uuids"/> is `null`.</exception>
    /// <exception cref="System.ArgumentNullException">Thrown if <paramref name="anchors"/> is `null`.</exception>
    public static OVRTask<bool> FetchAnchorsAsync(IEnumerable<Guid> uuids, IList<OVRAnchor> anchors,
        OVRSpace.StorageLocation location = OVRSpace.StorageLocation.Local, double timeout = 0.0)
    {
        if (uuids == null)
        {
            throw new ArgumentNullException(nameof(uuids));
        }

        if (anchors == null)
        {
            throw new ArgumentNullException(nameof(anchors));
        }

        return FetchAnchors(anchors, GetQueryInfo(uuids, location, timeout));
    }

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE

    /// <summary>
    /// Asynchronous method that fetches anchors with specifics uuids.
    /// </summary>
    /// <param name="localGroupUuid">Local group UUID from which to fetch anchors</param>
    /// <param name="anchors">IList that will get cleared and populated with the requested anchors.</param>s
    /// <param name="location">Storage location to query</param>
    /// <param name="timeout">Timeout in seconds for the query. Zero indicates the query does not timeout.</param>
    /// <remarks>Dispose of the returned <see cref="OVRTask{T}"/> if you don't use the results</remarks>
    /// <returns>An <see cref="OVRTask{T}"/> that will eventually let you test if the fetch was successful or not.
    /// If the result is true, then the <see cref="anchors"/> parameter has been populated with the requested anchors.</returns>
    /// <exception cref="System.ArgumentNullException">Thrown if <paramref name="localGroupUuid"/> is `null`.</exception>
    /// <exception cref="System.ArgumentNullException">Thrown if <paramref name="anchors"/> is `null`.</exception>
    public static OVRTask<bool> FetchAnchorsFromLocalGroupAsync(Guid localGroupUuid, IList<OVRAnchor> anchors,
        OVRSpace.StorageLocation location = OVRSpace.StorageLocation.Local, double timeout = 0.0)
    {
        if (localGroupUuid == null)
        {
            throw new ArgumentNullException(nameof(localGroupUuid));
        }

        if (anchors == null)
        {
            throw new ArgumentNullException(nameof(anchors));
        }

        return FetchAnchors2(anchors, GetQueryInfoFromLocalGroup(localGroupUuid, location, timeout));
    }

#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
    // Called by OVRManager event loop
    internal static void OnSpaceDiscoveryComplete(OVRDeserialize.SpaceDiscoveryCompleteData data)
    {
        TaskResult result;
        var task = OVRTask.GetExisting<TaskResult>(data.RequestId);
        if (task.TryGetInternalData<FetchTaskData>(out var taskData))
        {
            Telemetry.GetMarker(Telemetry.MarkerId.DiscoverSpaces, data.RequestId)
                ?.AddAnnotation(Telemetry.Annotation.ResultsCount, taskData.Anchors?.Count ?? 0);
            result = OVRResult.From(taskData.Anchors, (FetchResult)data.Result);
        }
        else
        {
            Debug.LogError(
                $"SpaceDiscovery completed but its task does not have an associated anchor List. This is likely a bug. RequestId={data.RequestId}, Result={data.Result}");
            result = OVRResult.From((List<OVRAnchor>)null, (FetchResult)data.Result);
        }

        Telemetry.SetAsyncResultAndSend(Telemetry.MarkerId.DiscoverSpaces, data.RequestId, data.Result);

        task.SetResult(result);
    }

    // Called by OVRManager event loop
    internal static void OnSpaceDiscoveryResultsAvailable(OVRDeserialize.SpaceDiscoveryResultsData data)
    {
        var requestId = data.RequestId;

        // never calls task.SetResult() as that completes the task
        var task = OVRTask.GetExisting<TaskResult>(requestId);
        if (!task.IsPending) return;

        if (!task.TryGetInternalData<FetchTaskData>(out var taskData))
            return;

        NativeArray<SpaceDiscoveryResult> results = default;
        Result result;
        int count;

        unsafe
        {
            result = RetrieveSpaceDiscoveryResults(requestId, null, 0, out count);
            if (!result.IsSuccess()) return;

            do
            {
                if (results.IsCreated)
                {
                    results.Dispose();
                }

                results = new NativeArray<SpaceDiscoveryResult>(count, Allocator.Temp);
                result = RetrieveSpaceDiscoveryResults(requestId, (SpaceDiscoveryResult*)results.GetUnsafePtr(),
                    results.Length, out count);
            } while (result == Result.Failure_InsufficientSize);
        }

        var startingIndex = taskData.Anchors.Count;

        using (results)
        {
            if (!result.IsSuccess())
            {
                return;
            }

            // always add to anchors, as the results are consumed
            for (var i = 0; i < count; i++)
            {
                var item = results[i];
                taskData.Anchors.Add(new OVRAnchor(item.Space, item.Uuid));
            }
        }

        // notify potential subscribers to the incremental results
        taskData.IncrementalResultsCallback?.Invoke(taskData.Anchors, startingIndex);
    }

    struct FetchTaskData
    {
        public List<OVRAnchor> Anchors;
        public Action<List<OVRAnchor>, int> IncrementalResultsCallback;
    }

    /// <summary>
    /// (Experimental) Fetch anchors matching a query.
    /// </summary>
    /// <remarks>
    /// This method queries for anchors that match the corresponding <paramref name="options"/>. This method is
    /// asynchronous; use the returned <see cref="OVRTask{TResult}"/> to check for completion.
    ///
    /// Anchors may be returned in batches. If <paramref name="incrementalResultsCallback"/> is not `null`, then this
    /// delegate is invoked whenever results become available prior to the completion of the entire operation. New anchors
    /// are appended to <paramref name="anchors"/>. The delegate receives a reference to <paramref name="anchors"/> and
    /// the starting index of the anchors that have been added. The parameters are:
    /// - `anchors`: The same `List` provided by <paramref name="anchors"/>.
    /// - `index`: The starting index of the newly available anchors
    /// </remarks>
    /// <param name="anchors">Container to store the results. The list is cleared before adding any anchors.</param>
    /// <param name="options">Options describing which anchors to fetch.</param>
    /// <param name="incrementalResultsCallback">(Optional) A callback invoked when incremental results are available.</param>
    /// <returns>An <see cref="OVRTask{TResult}"/> that can be used to track the asynchronous fetch.</returns>
    /// <exception cref="ArgumentNullException">Thrown if <paramref name="anchors"/> is `null`.</exception>
    public static OVRTask<TaskResult> FetchAnchorsAsync(
        List<OVRAnchor> anchors, FetchOptions options, Action<List<OVRAnchor>, int> incrementalResultsCallback = null)
    {
        if (anchors == null)
            throw new ArgumentNullException(nameof(anchors));

        anchors.Clear();

        var result = options.DiscoverSpaces(out var requestId);
        if (!result.IsSuccess())
        {
            return OVRTask.FromResult(OVRResult.From(anchors, (FetchResult)result));
        }

        var task = OVRTask.FromRequest<TaskResult>(requestId);
        task.SetInternalData(new FetchTaskData
        {
            Anchors = anchors,
            IncrementalResultsCallback = incrementalResultsCallback,
        });

        return task;
    }
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

    private static OVRTask<bool> FetchAnchors(IList<OVRAnchor> anchors, OVRPlugin.SpaceQueryInfo queryInfo)
    {
        if (anchors == null)
        {
            throw new ArgumentNullException(nameof(anchors));
        }

        anchors.Clear();

        var telemetryMarker = OVRTelemetry
            .Start((int)Telemetry.MarkerId.QuerySpaces)
            .AddAnnotation(Telemetry.Annotation.Timeout, (double)queryInfo.Timeout)
            .AddAnnotation(Telemetry.Annotation.MaxResults, (long)queryInfo.MaxQuerySpaces)
            .AddAnnotation(Telemetry.Annotation.StorageLocation, (long)queryInfo.Location);

        if (queryInfo is { FilterType: SpaceQueryFilterType.Components, ComponentsInfo: { Components: { Length: > 0 } } })
        {
            unsafe
            {
                var componentTypes = stackalloc long[queryInfo.ComponentsInfo.NumComponents];
                for (var i = 0; i < queryInfo.ComponentsInfo.NumComponents; i++)
                {
                    componentTypes[i] = (long)queryInfo.ComponentsInfo.Components[i];
                }
                telemetryMarker.AddAnnotation(Telemetry.Annotation.ComponentTypes, componentTypes,
                    queryInfo.ComponentsInfo.NumComponents);
            }
        }
        else if (queryInfo is { FilterType: SpaceQueryFilterType.Ids })
        {
            telemetryMarker.AddAnnotation(Telemetry.Annotation.UuidCount, (long)queryInfo.IdInfo.NumIds);
        }

        var result = QuerySpacesWithResult(queryInfo, out var requestId);
        Telemetry.SetSyncResult(telemetryMarker, requestId, result);

        if (!result.IsSuccess())
        {
            return OVRTask.FromResult(false);
        }

        var task = OVRTask.FromRequest<bool>(requestId);
        task.SetInternalData(anchors);
        return task;
    }

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE

    private static OVRTask<bool> FetchAnchors2(IList<OVRAnchor> anchors, OVRPlugin.SpaceQueryInfo2 queryInfo)
    {
        if (anchors == null)
        {
            throw new ArgumentNullException(nameof(anchors));
        }

        anchors.Clear();

        if (!OVRPlugin.QuerySpaces2(queryInfo, out var requestId))
        {
            return OVRTask.FromResult(false);
        }

        var task = OVRTask.FromRequest<bool>(requestId);
        task.SetInternalData(anchors);
        return task;
    }

#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

    internal static void OnSpaceQueryComplete(OVRDeserialize.SpaceQueryCompleteData data)
    {
        OVRTelemetryMarker? telemetryMarker = null;
        var task = OVRTask.GetExisting<bool>(data.RequestId);
        bool? taskResult = null;
        try
        {
            telemetryMarker =
                Telemetry.SetAsyncResult(Telemetry.MarkerId.QuerySpaces, data.RequestId, (long)data.Result);

            var requestId = data.RequestId;
            if (!task.IsPending)
            {
                return;
            }

            if (!task.TryGetInternalData<IList<OVRAnchor>>(out var anchors) || anchors == null)
            {
                taskResult = false;
                return;
            }

            if (!RetrieveSpaceQueryResults(requestId, out var rawResults, Allocator.Temp))
            {
                taskResult = false;
                return;
            }

            using (rawResults)
            {
                telemetryMarker?.AddAnnotation(Telemetry.Annotation.ResultsCount, (long)rawResults.Length);

                foreach (var result in rawResults)
                {
                    anchors.Add(new OVRAnchor(result.space, result.uuid));
                }

                taskResult = true;
            }
        }
        finally
        {
            telemetryMarker?.Send();
            if (taskResult.HasValue)
            {
                task.SetResult(taskResult.Value);
            }
        }
    }

    /// <summary>
    /// Creates a new spatial anchor.
    /// </summary>
    /// <remarks>
    /// Spatial anchor creation is asynchronous. This method initiates a request to create a spatial anchor at
    /// <paramref name="trackingSpacePose"/>. The returned <see cref="OVRTask{TResult}"/> can be awaited or used to
    /// track the completion of the request.
    ///
    /// If spatial anchor creation fails, the resulting <see cref="OVRAnchor"/> will be <see cref="OVRAnchor.Null"/>.
    /// </remarks>
    /// <param name="trackingSpacePose">The pose, in tracking space, at which you wish to create the spatial anchor.</param>
    /// <returns>A task which can be used to track completion of the request.</returns>
    public static OVRTask<OVRAnchor> CreateSpatialAnchorAsync(Pose trackingSpacePose)
        => CreateSpatialAnchor(new SpatialAnchorCreateInfo
        {
            BaseTracking = GetTrackingOriginType(),
            PoseInSpace = new Posef
            {
                Orientation = trackingSpacePose.rotation.ToFlippedZQuatf(),
                Position = trackingSpacePose.position.ToFlippedZVector3f(),
            },
            Time = GetTimeInSeconds(),
        }, out var requestId)
            ? OVRTask.FromRequest<OVRAnchor>(requestId)
            : OVRTask.FromResult(Null);

    /// <summary>
    /// Creates a new spatial anchor.
    /// </summary>
    /// <remarks>
    /// Spatial anchor creation is asynchronous. This method initiates a request to create a spatial anchor at
    /// <paramref name="transform"/>. The returned <see cref="OVRTask{TResult}"/> can be awaited or used to
    /// track the completion of the request.
    ///
    /// If spatial anchor creation fails, the resulting <see cref="OVRAnchor"/> will be <see cref="OVRAnchor.Null"/>.
    /// </remarks>
    /// <param name="transform">The transform at which you wish to create the spatial anchor.</param>
    /// <param name="centerEyeCamera">The `Camera` associated with the Meta Quest's center eye.</param>
    /// <returns>A task which can be used to track completion of the request.</returns>
    /// <exception cref="ArgumentNullException">Thrown when <paramref name="transform"/> is `null`.</exception>
    /// <exception cref="ArgumentNullException">Thrown when <paramref name="centerEyeCamera"/> is `null`.</exception>
    public static OVRTask<OVRAnchor> CreateSpatialAnchorAsync(Transform transform, Camera centerEyeCamera)
    {
        if (transform == null)
            throw new ArgumentNullException(nameof(transform));

        if (centerEyeCamera == null)
            throw new ArgumentNullException(nameof(centerEyeCamera));

        var pose = transform.ToTrackingSpacePose(centerEyeCamera);
        return CreateSpatialAnchorAsync(new Pose
        {
            position = pose.position,
            rotation = pose.orientation,
        });
    }

    #endregion

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
    /// <summary>
    /// (Experimental) Save this anchor.
    /// </summary>
    /// <remarks>
    /// :warning: This is an experimental API.
    ///
    /// This method persists the anchor so that it may be retrieved later, e.g., by using
    /// <see cref="FetchAnchorsAsync(List{OVRAnchor},FetchOptions,Action{List{OVRAnchor},int})"/>.
    ///
    /// This operation is asynchronous. Use the returned <see cref="OVRTask{TResult}"/> to track the result of the
    /// asynchronous operation.
    /// </remarks>
    /// <returns>An awaitable <see cref="OVRTask{TResult}"/> representing the asynchronous request.</returns>
    /// <seealso cref="FetchAnchorsAsync(List{OVRAnchor},FetchOptions,Action{List{OVRAnchor},int})"/>
    /// <seealso cref="EraseAsync()"/>
    public OVRTask<OVRResult<SaveResult>> SaveAsync() => s_deferredSaves.Add(this);

    /// <summary>
    /// (Experimental) Save a collection of anchors.
    /// </summary>
    /// <remarks>
    /// :warning: This is an experimental API.
    ///
    /// This method persists a collection of anchors so that they may be retrieved later.
    ///
    /// This operation is asynchronous. Use the returned <see cref="OVRTask{TResult}"/> to track the result of the
    /// asynchronous operation.
    /// </remarks>
    /// <param name="anchors">A collection of anchors to persist.</param>
    /// <returns>An awaitable <see cref="OVRTask{TResult}"/> representing the asynchronous request.</returns>
    /// <seealso cref="FetchAnchorsAsync(List{OVRAnchor},FetchOptions,Action{List{OVRAnchor},int})"/>
    /// <seealso cref="EraseAsync(IEnumerable{OVRAnchor},IEnumerable{Guid})"/>
    public static OVRTask<OVRResult<SaveResult>> SaveAsync(IEnumerable<OVRAnchor> anchors)
    {
        var collection = anchors.ToNonAlloc();
        var count = collection.GetCount();

        if (count == 0)
        {
            return OVRTask.FromResult(OVRResult.From(SaveResult.Success));
        }

        unsafe
        {
            var spaces = stackalloc ulong[count];

            var index = 0;
            foreach (var anchor in collection)
            {
                spaces[index++] = anchor.Handle;
            }

            return SaveSpacesAsync(spaces, count);
        }
    }

    internal static unsafe OVRTask<OVRResult<SaveResult>> SaveSpacesAsync(ulong* spaces, int count)
    {
        var telemetryMarker = OVRTelemetry
            .Start((int)Telemetry.MarkerId.SaveSpaces)
            .AddAnnotation(Telemetry.Annotation.SpaceCount, (long)count);

        var result = SaveSpaces(spaces, count, out var requestId);
        Telemetry.SetSyncResult(telemetryMarker, requestId, result);

        return result.IsSuccess()
            ? OVRTask.FromRequest<OVRResult<SaveResult>>(requestId)
            : OVRTask.FromResult(OVRResult.From((SaveResult)result));
    }

    // Invoked by OVRManager event loop
    internal static void OnSaveSpacesResult(OVRDeserialize.SpacesSaveResultData eventData)
        => Telemetry.SetAsyncResultAndSend(Telemetry.MarkerId.SaveSpaces, eventData.RequestId, (long)eventData.Result);

    /// <summary>
    /// (Experimental) Erases this anchor.
    /// </summary>
    /// <remarks>
    /// :warning: This is an experimental API.
    ///
    /// This method removes the anchor from persistent storage. Note this does not destroy the current instance.
    ///
    /// This operation is asynchronous. Use the returned <see cref="OVRTask{TResult}"/> to track the result of the
    /// asynchronous operation.
    /// </remarks>
    /// <returns>An awaitable <see cref="OVRTask{TResult}"/> representing the asynchronous request.</returns>
    /// <seealso cref="SaveAsync()"/>
    public OVRTask<OVRResult<EraseResult>> EraseAsync() => s_deferredErases.Add(this);

    /// <summary>
    /// (Experimental) Erase a collection of anchors.
    /// </summary>
    /// <remarks>
    /// :warning: This is an experimental API.
    ///
    /// This method removes a collection of anchors from persistent storage.
    ///
    /// This operation is asynchronous. Use the returned <see cref="OVRTask{TResult}"/> to track the result of the
    /// asynchronous operation.
    /// </remarks>
    /// <param name="anchors">(Optional) A collection of anchors to remove from persistent storage.</param>
    /// <param name="uuids">(Optional) A collection of uuids to remove from persistent storage.</param>
    /// <returns>An awaitable <see cref="OVRTask{TResult}"/> representing the asynchronous request.</returns>
    /// <exception cref="ArgumentException">Thrown if both <paramref name="anchors"/> and <paramref name="uuids"/> are `null`.</exception>
    /// <seealso cref="SaveAsync(IEnumerable{OVRAnchor})"/>
    public static OVRTask<OVRResult<EraseResult>> EraseAsync(
        IEnumerable<OVRAnchor> anchors,
        IEnumerable<Guid> uuids)
    {
        if (anchors == null && uuids == null)
            throw new ArgumentException($"One of {nameof(anchors)} or {nameof(uuids)} must not be null.");

        var anchorCollection = anchors.ToNonAlloc();
        var spaceCount = anchorCollection.GetCount();

        var uuidCollection = uuids.ToNonAlloc();
        var uuidCount = uuidCollection.GetCount();

        if (spaceCount == 0 && uuidCount == 0)
        {
            return OVRTask.FromResult(OVRResult.From(EraseResult.Success));
        }

        unsafe
        {
            var spaces = stackalloc ulong[spaceCount];
            var ids = stackalloc Guid[uuidCount];

            var index = 0;
            foreach (var anchor in anchorCollection)
            {
                spaces[index++] = anchor.Handle;
            }

            index = 0;
            foreach (var uuid in uuidCollection)
            {
                ids[index++] = uuid;
            }

            return EraseSpacesAsync(spaceCount, spaces, uuidCount, ids);
        }
    }

    internal static unsafe OVRTask<OVRResult<EraseResult>> EraseSpacesAsync(int spaceCount, ulong* spaces,
        int uuidCount, Guid* uuids)
    {
        var telemetryMarker = OVRTelemetry
            .Start((int)Telemetry.MarkerId.EraseSpaces)
            .AddAnnotation(Telemetry.Annotation.SpaceCount, (long)spaceCount)
            .AddAnnotation(Telemetry.Annotation.UuidCount, (long)uuidCount);

        var result = EraseSpaces((uint)spaceCount, spaces, (uint)uuidCount, uuids, out var requestId);
        Telemetry.SetSyncResult(telemetryMarker, requestId, result);

        return result.IsSuccess()
            ? OVRTask.FromRequest<OVRResult<EraseResult>>(requestId)
            : OVRTask.FromResult(OVRResult.From((EraseResult)result));
    }

    internal static void OnEraseSpacesResult(OVRDeserialize.SpacesEraseResultData eventData)
        => Telemetry.SetAsyncResultAndSend(Telemetry.MarkerId.EraseSpaces, eventData.RequestId, (long)eventData.Result);
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_return_to_room
    /// <summary>
    /// Invoked when the user enters a room.
    /// </summary>
    /// <remarks>
    /// The `Guid` provided is the UUID of the room anchor.
    /// </remarks>
    public static event Action<Guid> RoomEntered;

    /// <summary>
    /// Invoked when the user exits a room.
    /// </summary>
    /// <remarks>
    /// The `Guid` provided is the UUID of the room anchor.
    /// </remarks>
    public static event Action<Guid> RoomExited;

    // Invoked by OVRManager
    internal static void OnUserLocationChanged(Guid roomUuid, ReturnToRoomUserLocation location)
    {
        switch (location)
        {
            case ReturnToRoomUserLocation.Inside:
                RoomEntered?.Invoke(roomUuid);
                break;
            case ReturnToRoomUserLocation.Outside:
                RoomExited?.Invoke(roomUuid);
                break;
            default:
                Debug.LogError($"Unhandled location {location}");
                break;
        }
    }

    /// <summary>
    /// Possible results of <see cref="OVRAnchor.RequestReturnToRoom"/>
    /// </summary>
    [OVRResultStatus]
    public enum ReturnToRoomResult
    {
        /// <summary>
        /// The operation succeeded.
        /// </summary>
        Success = Result.Success,

        /// <summary>
        /// The operation failed.
        /// </summary>
        Failure = Result.Failure,
        /// <summary>
        /// Invalid data.
        /// </summary>
        FailureDataIsInvalid = Result.Failure_DataIsInvalid,

        /// <summary>
        /// Return to room is not supported.
        /// </summary>
        FailureUnsupported = Result.Failure_Unsupported,
    }

    /// <summary>
    /// Specify a set of rooms to which the user should be limited.
    /// </summary>
    /// <remarks>
    /// This communicates to the system that you wish for the user to remain within one or more rooms specified by
    /// <paramref name="roomUuids"/>. If the user is not in any such room, they will receive a prompt to return to
    /// a room.
    ///
    /// A "room" is an <see cref="OVRAnchor"/> that supports the <see cref="OVRRoomLayout"/> component. You can retrieve
    /// rooms using <see cref="FetchAnchorsAsync{T}"/> with `T`=<see cref="OVRRoomLayout"/>.
    /// </remarks>
    /// <param name="roomUuids">The UUIDs of the rooms to return to, or `null` to disable return-to-room functionality.</param>
    /// <returns>Returns the result of the request, see <see cref="ReturnToRoomResult"/>.</returns>
    public static OVRResult<ReturnToRoomResult> RequestReturnToRoom(IEnumerable<Guid> roomUuids)
    {
        var collection = roomUuids.ToNonAlloc();
        unsafe
        {
            var uuids = stackalloc Guid[collection.GetCount()];
            uint count = 0;
            foreach (var uuid in collection)
            {
                uuids[count++] = uuid;
            }

            return OVRResult.From((ReturnToRoomResult)OVRPlugin.RequestReturnToRoom(count, uuids));
        }
    }
#endif

    internal ulong Handle { get; }

    /// <summary>
    /// Unique Identifier representing the anchor.
    /// </summary>
    public Guid Uuid { get; }

    internal OVRAnchor(ulong handle, Guid uuid)
    {
        Handle = handle;
        Uuid = uuid;
    }

    /// <summary>
    /// Gets the anchor's component of a specific type.
    /// </summary>
    /// <typeparam name="T">The type of the component.</typeparam>
    /// <returns>The requested component.</returns>
    /// <remarks>Make sure the anchor supports the specified type of component using <see cref="SupportsComponent{T}"/></remarks>
    /// <exception cref="InvalidOperationException">Thrown if the anchor doesn't support the specified type of component.</exception>
    /// <seealso cref="TryGetComponent{T}"/>
    /// <seealso cref="SupportsComponent{T}"/>
    public T GetComponent<T>() where T : struct, IOVRAnchorComponent<T>
    {
        if (!TryGetComponent<T>(out var component))
        {
            throw new InvalidOperationException($"Anchor {Uuid} does not have component {typeof(T).Name}");
        }

        return component;
    }

    /// <summary>
    /// Tries to get the anchor's component of a specific type.
    /// </summary>
    /// <param name="component">The requested component, as an <c>out</c> parameter.</param>
    /// <typeparam name="T">The type of the component.</typeparam>
    /// <returns>Whether or not the request succeeded. It may fail if the anchor doesn't support this type of component.</returns>
    /// <seealso cref="GetComponent{T}"/>
    public bool TryGetComponent<T>(out T component) where T : struct, IOVRAnchorComponent<T>
    {
        component = default;
        if (!GetSpaceComponentStatusInternal(Handle, component.Type, out _, out _).IsSuccess())
        {
            return false;
        }

        component = component.FromAnchor(this);
        return true;
    }

    /// <summary>
    /// Tests whether or not the anchor supports a specific type of component.
    /// </summary>
    /// <remarks>
    /// For performance reasons, we use xrGetSpaceComponentStatusFB, which can
    /// result in an error in the logs when the component is not available.
    ///
    /// This error does not have impact on the control flow. The alternative method,
    /// <seealso cref="GetSupportedComponents(List{SpaceComponentType})"/> avoids
    /// this error reporting, but does have performance constraints.
    /// </remarks>
    /// <typeparam name="T">The type of the component.</typeparam>
    /// <returns>Whether or not the specified type of component is supported.</returns>
    public bool SupportsComponent<T>() where T : struct, IOVRAnchorComponent<T>
        => GetSpaceComponentStatusInternal(Handle, default(T).Type, out _, out _).IsSuccess();

    /// <summary>
    /// Get all the supported components of an anchor.
    /// </summary>
    /// <param name="components">The list to populate with the supported components. The list is cleared first.</param>
    /// <returns>`True` if the supported components could be retrieved, otherwise `False`.</returns>
    public bool GetSupportedComponents(List<SpaceComponentType> components)
    {
        components.Clear();

        unsafe
        {
            if (!EnumerateSpaceSupportedComponents(Handle, 0, out var count, null).IsSuccess())
                return false;

            var buffer = stackalloc SpaceComponentType[(int)count];
            if (!EnumerateSpaceSupportedComponents(Handle, count, out count, buffer).IsSuccess())
                return false;

            for (uint i = 0; i < count; i++)
            {
                components.Add(buffer[i]);
            }

            return true;
        }
    }

    public bool Equals(OVRAnchor other) => Handle.Equals(other.Handle) && Uuid.Equals(other.Uuid);
    public override bool Equals(object obj) => obj is OVRAnchor other && Equals(other);
    public static bool operator ==(OVRAnchor lhs, OVRAnchor rhs) => lhs.Equals(rhs);
    public static bool operator !=(OVRAnchor lhs, OVRAnchor rhs) => !lhs.Equals(rhs);
    public override int GetHashCode() => unchecked(Handle.GetHashCode() * 486187739 + Uuid.GetHashCode());
    public override string ToString() => Uuid.ToString();

    /// <summary>
    /// Disposes of an anchor.
    /// </summary>
    /// <remarks>
    /// Calling this method will destroy the anchor so that it won't be managed by internal systems until
    /// the next time it is fetched again.
    /// </remarks>
    public void Dispose() => OVRPlugin.DestroySpace(Handle);

    struct DeferredValue
    {
        public OVRTask<bool> Task;
        public bool EnabledDesired;
        public ulong RequestId;
        public double Timeout;
        public float StartTime;
    }

    struct DeferredKey : IEquatable<DeferredKey>
    {
        public ulong Space;
        public SpaceComponentType ComponentType;
        public static DeferredKey FromEvent(OVRDeserialize.SpaceSetComponentStatusCompleteData eventData) => new()
        {
            Space = eventData.Space,
            ComponentType = eventData.ComponentType,
        };

        public bool Equals(DeferredKey other) => Space == other.Space && ComponentType == other.ComponentType;
        public override bool Equals(object obj) => obj is DeferredKey other && Equals(other);
        public override int GetHashCode() => unchecked(Space.GetHashCode() * 486187739 + ((int)ComponentType).GetHashCode());
    }

    static readonly Dictionary<DeferredKey, List<DeferredValue>> _deferredTasks = new();

    [RuntimeInitializeOnLoadMethod]
    internal static void Init()
    {
        _deferredTasks.Clear();
        Telemetry.OnInit();
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_spatial_entity_persistence
        s_deferredSaves.Clear();
        s_deferredErases.Clear();
#endif
    }

    internal static OVRTask<bool> CreateDeferredSpaceComponentStatusTask(ulong space, SpaceComponentType componentType, bool enabledDesired, double timeout)
    {
        var key = new DeferredKey
        {
            Space = space,
            ComponentType = componentType
        };

        if (!_deferredTasks.TryGetValue(key, out var list))
        {
            list = OVRObjectPool.List<DeferredValue>();
            _deferredTasks.Add(key, list);
        }

        var task = OVRTask.FromGuid<bool>(Guid.NewGuid());

        list.Add(new DeferredValue
        {
            EnabledDesired = enabledDesired,
            Task = task,
            Timeout = timeout,
            StartTime = Time.realtimeSinceStartup,
        });

        return task;
    }

    internal static void OnSpaceSetComponentStatusComplete(OVRDeserialize.SpaceSetComponentStatusCompleteData eventData)
    {
        var key = DeferredKey.FromEvent(eventData);
        if (!_deferredTasks.TryGetValue(key, out var list)) return;

        try
        {
            var isEnabled = eventData.Enabled != 0;
            for (var i = 0; i < list.Count; i++)
            {
                var value = list[i];
                var task = value.Task;
                bool? result = null;

                if (eventData.RequestId == value.RequestId)
                {
                    // If this result was initiated by us, then use that value
                    result = eventData.Result >= 0;
                }
                else if (isEnabled == value.EnabledDesired)
                {
                    // We're done!
                    result = true;
                }
                // Check to see if there is any other change pending
                else if (!GetSpaceComponentStatus(eventData.Space, eventData.ComponentType, out _,
                        out var changePending))
                {
                    result = false;
                }
                // If there's no other change pending, then try to change the component status
                else if (!changePending)
                {
                    var timeout = value.Timeout;
                    if (timeout > 0)
                    {
                        // Subtract elapsed time
                        timeout -= Time.realtimeSinceStartup - value.StartTime;
                        if (timeout <= 0)
                        {
                            result = false;
                        }
                    }

                    if (result == null)
                    {
                        if (SetSpaceComponentStatus(eventData.Space, eventData.ComponentType, value.EnabledDesired,
                                timeout, out var requestId))
                        {
                            value.RequestId = requestId;
                            list[i] = value;
                        }
                        else
                        {
                            result = false;
                        }
                    }
                }

                if (result.HasValue)
                {
                    list.RemoveAt(i--);
                    task.SetResult(result.Value);
                }
            }
        }
        finally
        {
            if (list.Count == 0)
            {
                OVRObjectPool.Return(list);
                _deferredTasks.Remove(key);
            }
        }
    }
}
