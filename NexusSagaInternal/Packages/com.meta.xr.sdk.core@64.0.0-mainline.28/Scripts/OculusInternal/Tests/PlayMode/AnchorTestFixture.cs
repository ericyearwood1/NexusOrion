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
using System.Runtime.InteropServices;
using System.Text;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.TestTools;
using static OVRPlugin;
using Object = UnityEngine.Object;

[TestFixture]
internal unsafe class AnchorTestFixture : OVRPluginPlayModeTest
{
    private Plugin55 _plugin55;

    private Plugin72 _plugin72;

    private Plugin79 _plugin79;

    private Plugin92 _plugin92;

    private Plugin83 _plugin83;

    private Plugin86 _plugin86;

    private Plugin12 _plugin12;

    private Plugin93 _plugin93;

    private ulong _nextRequestId;

    protected readonly Dictionary<ulong, Space> Spaces = new Dictionary<ulong, Space>();
    protected Dictionary<ulong, HashSet<Space>> SpaceContainers => _plugin72.SpaceContainers;

    protected bool PlaneTrackingSupported;

    protected OVRCameraRig CameraRig { get; private set; }

    protected internal interface IComponent
    {
        SpaceComponentType ComponentType { get; }
    }

    protected internal enum OvrSpaceFBType
    {
        unknown = -1,
        spatial_anchor = 0,
        plane_anchor = 1,
        room_entity = 2,
    };

    protected internal enum XR_SPACE_COMPONENT_TYPE
    {
        // aligned with OVRPlugin.SpaceComponentType
        LOCATABLE = 0,
        STORABLE = 1,
        SHARABLE = 2,
        BOUNDED_2D = 3,
        BOUNDED_3D = 4,
        SEMANTIC_LABELS = 5,
        ROOM_LAYOUT = 6,
        SPACE_CONTAINER = 7,
        TRIANGLE_MESH = 1000269000,
#if OVR_INTERNAL_CODE // Room entity info
        ROOM_LABEL = 1000293001,
#endif
        // extra
        HEIGHT_MAP = 10,
        USER_LOCKS = 11,
    }

#if !OVR_INTERNAL_CODE
    public struct SpaceQueryInfo2
    {
        public SpaceQueryType QueryType;
        public int MaxQuerySpaces;
        public double Timeout;
        public SpaceStorageLocation Location;
        public SpaceQueryActionType ActionType;
        public SpaceQueryFilterType FilterType;
        public SpaceFilterInfoIds IdInfo;
        public SpaceFilterInfoComponents ComponentsInfo;
    }
#endif

    protected internal class Space
    {
        private readonly List<IComponent> _components = new List<IComponent>();
        private readonly HashSet<SpaceComponentType> _unsupportedComponents = new HashSet<SpaceComponentType>();

        public Space(OvrSpaceFBType type) : this(Guid.NewGuid(), type)
        { }

        public Space(Guid uuid, OvrSpaceFBType type)
        {
            Uuid = uuid;
            SetSupportedComponents(type);
        }

        private void SetSupportedComponents(OvrSpaceFBType type)
        {
            SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.LOCATABLE);

            switch (type)
            {
                case OvrSpaceFBType.spatial_anchor:
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.STORABLE);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.SHARABLE);
                    break;
                case OvrSpaceFBType.plane_anchor:
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.STORABLE);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.BOUNDED_2D);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.BOUNDED_3D);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.SEMANTIC_LABELS);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.HEIGHT_MAP);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.USER_LOCKS);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.TRIANGLE_MESH);
                    break;
                case OvrSpaceFBType.room_entity:
                    SupportedComponents.Remove(XR_SPACE_COMPONENT_TYPE.LOCATABLE);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.STORABLE);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.ROOM_LAYOUT);
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.SPACE_CONTAINER);
#if OVR_INTERNAL_CODE // Room entity info
                    SupportedComponents.Add(XR_SPACE_COMPONENT_TYPE.ROOM_LABEL);
#endif
                    break;
            }
        }

        public T AddComponent<T>() where T : IComponent
        {
            var component = _components.OfType<T>().FirstOrDefault();
            if (component == null)
            {
                component = Activator.CreateInstance<T>();
                _components.Add(component);
            }

            return component;
        }

        public bool RemoveComponent<T>() where T : IComponent
        {
            foreach (var component in _components)
            {
                if (component is T)
                {
                    return _components.Remove(component);
                }
            }

            return false;
        }

        public void SetEnabled<T>(bool enabled) where T : IComponent
        {
            if (enabled)
            {
                AddComponent<T>();
            }
            else
            {
                RemoveComponent<T>();
            }
        }

        public void AddUnsupportedComponent<T>() where T : IComponent
        {
            var component = Activator.CreateInstance<T>();
            _unsupportedComponents.Add(component.ComponentType);
        }

        public bool TryGetComponent<T>(out T component) where T : IComponent
        {
            component = GetComponent<T>();
            return component != null;
        }

        public T GetComponent<T>() where T : IComponent => _components.OfType<T>().FirstOrDefault();

        public bool HasComponent<T>() where T : IComponent => GetComponent<T>() != null;

        public bool HasComponent(SpaceComponentType componentType)
            => _components.Any(c => c.ComponentType == componentType);

        public bool SupportsComponent(SpaceComponentType componentType) =>
            !_unsupportedComponents.Contains(componentType);

        public readonly Guid Uuid = Guid.NewGuid();

        public ulong Handle => (ulong)GetHashCode();

        public SpaceQueryResult ToQueryResult() => new SpaceQueryResult
        {
            space = Handle,
            uuid = Uuid
        };

#if OVR_INTERNAL_CODE // XR_META_spatial_entity_discovery
        public SpaceDiscoveryResult ToDiscoveryResult() => new SpaceDiscoveryResult
        {
            Space = Handle,
            Uuid = Uuid
        };
#endif

        public static implicit operator ulong(Space space) => space.Handle;

        public List<XR_SPACE_COMPONENT_TYPE> SupportedComponents { get; } = new List<XR_SPACE_COMPONENT_TYPE>();
    }

    protected internal class SpatialAnchor : Space
    {
        public SpatialAnchor() : base(OvrSpaceFBType.spatial_anchor)
        { }

        public SpatialAnchor(Guid uuid) : base(uuid, OvrSpaceFBType.spatial_anchor)
        { }
    }

    protected class Sharable : IComponent
    {
        public SpaceComponentType ComponentType => SpaceComponentType.Sharable;
    }

    protected class Storable : IComponent
    {
        public SpaceComponentType ComponentType => SpaceComponentType.Storable;
    }

    protected class Locatable : IComponent
    {
        public SpaceComponentType ComponentType => SpaceComponentType.Locatable;

        public SpaceLocationf SpaceLocation = new SpaceLocationf
        {
            locationFlags =
                SpaceLocationFlags.PositionTracked |
                SpaceLocationFlags.OrientationTracked |
                SpaceLocationFlags.PositionValid |
                SpaceLocationFlags.OrientationValid,
            pose = Posef.identity
        };
    }

    protected class Bounded2D : IComponent
    {
        public SpaceComponentType ComponentType => SpaceComponentType.Bounded2D;
        public Vector2[] Boundary = Array.Empty<Vector2>();
        public Rectf BoundingBox;
    }

    protected class Bounded3D : IComponent
    {
        public SpaceComponentType ComponentType => SpaceComponentType.Bounded3D;
        public Boundsf BoundingBox;
    }

    protected class SemanticLabels : IComponent
    {
        public SpaceComponentType ComponentType => SpaceComponentType.SemanticLabels;
        public string[] Labels = Array.Empty<string>();

        public override string ToString() => Labels == null
            ? ""
            : OVRSemanticClassification.ValidateAndUpgradeLabels(
                string.Join(",", Labels));
    }

    protected class SpaceContainer : IComponent
    {
        public SpaceComponentType ComponentType => SpaceComponentType.SpaceContainer;
    }

    protected class RoomLayout : IComponent
    {
        public SpaceComponentType ComponentType => SpaceComponentType.RoomLayout;
        public Space[] Walls = Array.Empty<Space>();
        public Space Floor;
        public Space Ceiling;

        public IEnumerable<Space> Spaces
        {
            get
            {
                yield return Floor;
                yield return Ceiling;
                foreach (var wall in Walls)
                {
                    yield return wall;
                }
            }
        }

        public IEnumerable<ulong> Handles => Spaces.Select(s => s.Handle);

        public IEnumerable<Guid> Uuids
        {
            get
            {
                yield return Floor.Uuid;
                yield return Ceiling.Uuid;
                foreach (var wall in Walls)
                {
                    yield return wall.Uuid;
                }
            }
        }

        public OVRPlugin.RoomLayout ToRoomLayout() => new OVRPlugin.RoomLayout
        {
            ceilingUuid = Ceiling?.Uuid ?? Guid.Empty,
            floorUuid = Floor?.Uuid ?? Guid.Empty,
            wallUuids = Walls.Select(wall => wall.Uuid).ToArray()
        };
    }

#if OVR_INTERNAL_CODE // Room entity info
    protected class RoomLabel : IComponent
    {
        public SpaceComponentType ComponentType => SpaceComponentType.RoomLabel;
        public string Value;
    }
#endif

    protected internal struct ComponentStatusChangeRequest
    {
        public Space Space;
        public SpaceComponentType ComponentType;
        public Bool Enable;
        public double Timeout;
    }

    public struct SceneCaptureRequest
    {
        public byte[] Bytes;

        public string Value => Bytes == null ? null : Encoding.ASCII.GetString(Bytes);
    }

    protected internal Dictionary<ulong, SceneCaptureRequest> SceneCaptureRequests => _plugin72.SceneCaptureRequests;

    protected internal IReadOnlyDictionary<ulong, ComponentStatusChangeRequest> ComponentStatusChangeRequests
        => _plugin72.ComponentStatusChangeRequests;

#if OVR_INTERNAL_CODE
    protected IReadOnlyDictionary<ulong, DiscoveryInfo> DiscoveryQueries
        => _plugin92.DiscoveryQueries;

    protected Dictionary<ulong, ulong[]> SaveSpacesRequests => _plugin92.SaveSpacesRequests;

    protected Dictionary<ulong, Plugin92.EraseSpacesInfo> EraseSpacesRequests => _plugin92.EraseSpacesRequests;
#endif

    protected IReadOnlyDictionary<ulong, SpaceQueryInfo2> SpaceQueries
        => _plugin86.SpaceQueries;

    protected IReadOnlyDictionary<ulong, SpatialAnchorCreateInfo> SpatialAnchorCreationRequests
        => _plugin72.SpatialAnchorCreationRequests;

    protected void EnqueueEvent<T>(OVRPlugin.EventType type, T value) where T : struct
        => _plugin55.EnqueueEvent(type, value);

    internal void SetNodePose(Node nodeId, PoseStatef pose) => _plugin12.SetNodePose(nodeId, pose);

    protected internal void HandleComponentStatusChangeRequest(
        OVRDeserialize.SpaceSetComponentStatusCompleteData data)
        => _plugin72.HandleComponentStatusChangeRequest(data);

    protected internal void HandleSceneCaptureRequest(OVRDeserialize.SceneCaptureCompleteData data)
        => _plugin72.HandleSceneCaptureRequest(data);

#if OVR_INTERNAL_CODE
    protected void HandleDiscoverSpaces(ulong requestId, SpaceDiscoveryResult[] results)
        => _plugin92.HandleDiscoverSpaces(requestId, results);

    protected void AddSpaceDiscoveryResults(ulong requestId, SpaceDiscoveryResult[] results, bool isFinal)
        => _plugin92.AddSpaceDiscoveryResults(requestId, results, isFinal);
#endif

    protected Result HandleQuerySpaces(ref SpaceQueryInfo queryInfo, out ulong requestId)
        => _plugin86.HandleQuerySpaces(ref queryInfo, out requestId);

    protected void HandleSpaceQuery(ulong requestId, SpaceQueryResult[] results)
        => _plugin86.HandleSpaceQuery(requestId, results);

    protected Result HandleRetrieveSpaceQueryResults(ref ulong requestId,
        uint resultCapacityInput, ref uint resultCountOutput, IntPtr resultsBuffer)
        => _plugin86.HandleRetrieveSpaceQueryResults(
            ref requestId, resultCapacityInput, ref resultCountOutput, resultsBuffer);

    protected void HandleSpatialAnchorCreationRequest(
        OVRDeserialize.SpatialAnchorCreateCompleteData data, Space space)
        => _plugin72.HandleSpatialAnchorCreationRequest(data, space);

    protected internal IReadOnlyDictionary<ulong, ulong[]> SaveSpaceListRequests => _plugin79.SaveSpaceListRequests;
    protected internal IReadOnlyDictionary<ulong, UInt64> EraseQueries => _plugin72.EraseQueries;

    protected bool CreateSpatialAnchorSucceed
    {
        get => _plugin72.CreateSpatialAnchorSucceed;
        set => _plugin72.CreateSpatialAnchorSucceed = value;
    }

    protected bool SetComponentStatusSucceed
    {
        get => _plugin72.SetComponentStatusSucceed;
        set => _plugin72.SetComponentStatusSucceed = value;
    }

    protected bool EraseSpaceSucceed
    {
        get => _plugin72.EraseSpaceSucceed;
        set => _plugin72.EraseSpaceSucceed = value;
    }

    protected bool SpaceQuerySucceed
    {
        get => _plugin86.SpaceQuerySucceed;
        set => _plugin86.SpaceQuerySucceed = value;
    }

    protected bool SaveSpaceListSucceed
    {
        get => _plugin79.SaveSpaceListSucceed;
        set => _plugin79.SaveSpaceListSucceed = value;
    }

    protected bool ShareSpaceSucceed
    {
        get => _plugin79.ShareSpaceSucceed;
        set => _plugin79.ShareSpaceSucceed = value;
    }

    public struct ShareQueryData
    {
        public ulong[] Spaces;
        public ulong[] UserHandles;
    }

    protected internal IReadOnlyDictionary<ulong, ShareQueryData> ShareQueries => _plugin79.ShareQueries;

    public IReadOnlyDictionary<ulong, ulong> SpaceUsers => _plugin79.SpaceUsers;

    protected internal void HandleSaveSpaceListRequest(ulong requestId, Result result) =>
        _plugin79.HandleSaveSpaceListRequest(requestId, result);

    protected internal void HandleShareQuery(ulong requestId, OVRPlugin.Result result) =>
        _plugin79.HandleShareQuery(requestId, result);

    protected internal void HandleEraseQuery(ulong requestId, OVRPlugin.Result result, Guid uuid,
        OVRPlugin.SpaceStorageLocation spaceStorageLocation) =>
        _plugin72.HandleEraseQuery(requestId, result, uuid, spaceStorageLocation);

#if OVR_INTERNAL_CODE
    protected void HandleAllDiscoverSpaces()
    {
        var queries = DiscoveryQueries
            .Select(kvp => (kvp.Key, kvp.Value))
            .ToArray();
        foreach (var (requestId, query) in queries)
        {
            HandleDiscoverSpaces(requestId, query);
        }
    }

    protected unsafe void HandleDiscoverSpaces(ulong requestId, DiscoveryInfo info)
    {
        // manually filter the spaces, then create and inject the results
        var results = new List<SpaceDiscoveryResult>();

        foreach (var space in Spaces.Values)
        {
            if (info.Filters.Length == 0)
            {
                results.Add(space.ToDiscoveryResult());
                continue;
            }

            var shouldAddSpace = true;
            foreach (var filter in info.Filters)
            {
                if (filter.Type == SpaceDiscoveryFilterType.Component)
                {
                    var componentFilter = (DiscoveryFilterInfoComponents)filter;
                    if (!space.HasComponent(componentFilter.Component))
                    {
                        shouldAddSpace = false;
                        break;
                    }
                }
                else if (filter.Type == SpaceDiscoveryFilterType.Ids)
                {
                    var idsFilter = (DiscoveryFilterInfoIds)filter;
                    var anyIdsMatch = false;
                    foreach (var uuid in idsFilter.Ids)
                    {
                        if (space.Uuid == uuid)
                        {
                            anyIdsMatch = true;
                            break;
                        }
                    }

                    if (!anyIdsMatch)
                    {
                        shouldAddSpace = false;
                        break;
                    }
                }
            }

            if (shouldAddSpace)
            {
                results.Add(space.ToDiscoveryResult());
            }
        }

        HandleDiscoverSpaces(requestId, results.ToArray());
    }
#endif

    protected internal void HandleAllSpaceQueries()
    {
        // This creates a copy of the space queries so we can mutate the dictionary as we iterate
        // its entries.
        var queries = SpaceQueries
            .Select(kvp => (kvp.Key, kvp.Value))
            .ToArray();
        foreach (var (requestId, query) in queries)
        {
            HandleSpaceQuery(requestId, query);
        }
    }

    protected internal void HandleAllSpatialAnchorCreationRequests()
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

    protected static SpaceQueryInfo2 ToSpaceQueryInfo2(ref SpaceQueryInfo queryInfo)
    {
        return new SpaceQueryInfo2
        {
            QueryType = queryInfo.QueryType,
            MaxQuerySpaces = queryInfo.MaxQuerySpaces,
            Timeout = queryInfo.Timeout,
            Location = queryInfo.Location,
            ActionType = queryInfo.ActionType,
            FilterType = queryInfo.FilterType,
            IdInfo = queryInfo.IdInfo,
            ComponentsInfo = queryInfo.ComponentsInfo,
        };
    }

    protected void HandleSpaceQuery(ulong requestId, SpaceQueryInfo query)
    {
        var query2 = ToSpaceQueryInfo2(ref query);
        HandleSpaceQuery(requestId, query2);
    }

    protected void HandleSpaceQuery(ulong requestId, SpaceQueryInfo2 query)
    {
        switch (query.FilterType)
        {
            case SpaceQueryFilterType.Ids:
            {
                var ids = query.IdInfo.Ids.Take(query.IdInfo.NumIds);
                HandleSpaceQuery(requestId, Spaces.Values
                    .Where(space => ids.Contains(space.Uuid))
                    .Select(space => space.ToQueryResult())
                    .Take(query.MaxQuerySpaces)
                    .ToArray());
                break;
            }
            case SpaceQueryFilterType.Components:
            {
                Assert.AreEqual(1, query.ComponentsInfo.NumComponents);
                HandleSpaceQuery(requestId, Spaces.Values
                    .Where(space => space.HasComponent(query.ComponentsInfo.Components[0]))
                    .Select(space => space.ToQueryResult())
                    .Take(query.MaxQuerySpaces)
                    .ToArray());
                break;
            }
            default:
            {
                Assert.Fail($"Unexpected filter type {query.FilterType}");
                break;
            }
        }
    }

    internal void AddSpace(Space space) => Spaces.Add(space.Handle, space);

    internal Space CreateSpace(OvrSpaceFBType type)
    {
        var space = new Space(type);
        AddSpace(space);
        return space;
    }

    protected OVRCameraRig CreateDefaultCameraRig()
    {
        var go = new GameObject("OVRCameraRig",
            typeof(OVRCameraRig),
            typeof(OVRManager),
            typeof(OVRHeadsetEmulator),
            typeof(AudioListener));

        CameraRig = go.GetComponent<OVRCameraRig>();
        return CameraRig;
    }

    protected virtual Plugin1 CreatePlugin1() => new Plugin1();
    protected virtual Plugin12 CreatePlugin12() => new Plugin12();
    protected virtual Plugin55 CreatePlugin55() => new Plugin55();
    protected virtual Plugin64 CreatePlugin64(AnchorTestFixture fixture) => new Plugin64(fixture);
    protected virtual Plugin65 CreatePlugin65(AnchorTestFixture fixture) => new Plugin65(fixture);
    protected virtual Plugin72 CreatePlugin72(AnchorTestFixture fixture) => new Plugin72(fixture);
    protected virtual Plugin79 CreatePlugin79(AnchorTestFixture fixture) => new Plugin79(fixture);
    protected virtual Plugin83 CreatePlugin83() => new Plugin83();
    protected virtual Plugin86 CreatePlugin86(AnchorTestFixture fixture) => new Plugin86(fixture);
    protected virtual Plugin92 CreatePlugin92(AnchorTestFixture fixture) => new Plugin92(fixture);
    protected virtual Plugin93 CreatePlugin93(AnchorTestFixture fixture) => new Plugin93(fixture);

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        Mock();
    }

    public void Mock()
    {
        _nextRequestId = 1;
        PlaneTrackingSupported = false;

        // Setup mocks
        OVRP_1_0_0.mockObj = CreatePlugin1();
        OVRP_1_12_0.mockObj = _plugin12 = CreatePlugin12();
        OVRP_1_55_1.mockObj = _plugin55 = CreatePlugin55();
        OVRP_1_64_0.mockObj = CreatePlugin64(this);
        OVRP_1_65_0.mockObj = CreatePlugin65(this);
        OVRP_1_72_0.mockObj = _plugin72 = CreatePlugin72(this);
        OVRP_1_79_0.mockObj = _plugin79 = CreatePlugin79(this);
        OVRP_1_83_0.mockObj = _plugin83 = CreatePlugin83();
        OVRP_1_86_0.mockObj = _plugin86 = CreatePlugin86(this);
        OVRP_1_92_0.mockObj = _plugin92 = CreatePlugin92(this);
        OVRP_1_93_0.mockObj = _plugin93 = CreatePlugin93(this);
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        Unmock();
        if (CameraRig)
        {
            Object.DestroyImmediate(CameraRig.gameObject);
        }

        yield return base.UnityTearDown();
    }

    public void Unmock()
    {
        OVRP_1_93_0.mockObj = new OVRP_1_93_0_TEST();
        OVRP_1_92_0.mockObj = new OVRP_1_92_0_TEST();
        OVRP_1_86_0.mockObj = new OVRP_1_86_0_TEST();
        OVRP_1_82_0.mockObj = new OVRP_1_82_0_TEST();
        OVRP_1_83_0.mockObj = new OVRP_1_83_0_TEST();
        OVRP_1_79_0.mockObj = new OVRP_1_79_0_TEST();
        OVRP_1_72_0.mockObj = new OVRP_1_72_0_TEST();
        OVRP_1_64_0.mockObj = new OVRP_1_64_0_TEST();
        OVRP_1_65_0.mockObj = new OVRP_1_65_0_TEST();
        OVRP_1_55_1.mockObj = new OVRP_1_55_1_TEST();
        OVRP_1_12_0.mockObj = new OVRP_1_12_0_TEST();
        OVRP_1_0_0.mockObj = new OVRP_1_0_0_TEST();

        Spaces.Clear();
    }

    public void SetOVRInputState(OVRInput.Button currentState) => _plugin83.CurrentState = currentState;

    private static void CopyToPtr<T>(IEnumerable<T> source, IntPtr ptr) where T : struct
    {
        if (source == null) return;

        var offset = 0;
        foreach (var item in source)
        {
            Marshal.StructureToPtr(item, ptr + offset, fDeleteOld: false);
            offset += Marshal.SizeOf<T>();
        }
    }

    private ulong GenerateRequestId() => _nextRequestId++;

    protected class Plugin1 : OVRP_1_0_0_TEST
    {
        public override TrackingOrigin ovrp_GetTrackingOriginType()
        {
            return TrackingOrigin.Stage;
        }
    }

    protected class Plugin12 : OVRP_1_12_0_TEST
    {
        private readonly Dictionary<Node, PoseStatef> _poseDictionary = new();
        public void SetNodePose(Node nodeId, PoseStatef pose)
        {
            _poseDictionary[nodeId] = pose;
        }

        public override PoseStatef ovrp_GetNodePoseState(Step stepId, Node nodeId)
        {
            return _poseDictionary.TryGetValue(nodeId, out var value) ? value : PoseStatef.identity;
        }
    }

    protected class Plugin55 : OVRP_1_55_1_TEST
    {
        // Same as OVRPlugin
        private const int EventDataBufferSize = 4000;

        private readonly IntPtr _buffer;

        private Queue<EventDataBuffer> _events = new Queue<EventDataBuffer>();

        public Plugin55() => _buffer = Marshal.AllocHGlobal(EventDataBufferSize);

        ~Plugin55() => Marshal.FreeHGlobal(_buffer);

        public override Result ovrp_PollEvent2(ref OVRPlugin.EventType eventType, ref IntPtr eventData)
        {
            if (_events.Count == 0) return Result.Success_EventUnavailable;

            var @event = _events.Dequeue();
            eventType = @event.EventType;
            Marshal.Copy(@event.EventData, 0, _buffer, EventDataBufferSize);
            eventData = _buffer;

            return Result.Success;
        }

        public void EnqueueEvent<T>(OVRPlugin.EventType type, T value) where T : struct
        {
            var @event = new EventDataBuffer
            {
                EventType = type,
                EventData = new byte[EventDataBufferSize]
            };

            fixed (byte* ptr = @event.EventData)
            {
                Marshal.StructureToPtr(value, new IntPtr(ptr), fDeleteOld: false);
            }

            _events.Enqueue(@event);
        }
    }

    protected class Plugin64 : OVRP_1_64_0_TEST
    {
        private readonly AnchorTestFixture _context;

        public Plugin64(AnchorTestFixture context) => _context = context;

        public override Result ovrp_LocateSpace(ref Posef location, ref ulong space,
            TrackingOrigin trackingOrigin)
        {
            if (!_context.Spaces.TryGetValue(space, out var anchor) ||
                !anchor.TryGetComponent<Locatable>(out var locatable))
            {
                return Result.Failure;
            }

            location = locatable.SpaceLocation.pose;
            return Result.Success;
        }
    }

    protected class Plugin65 : OVRP_1_65_0_TEST
    {
        private readonly AnchorTestFixture _context;

        public Plugin65(AnchorTestFixture context) => _context = context;

        public override Result ovrp_DestroySpace(ref UInt64 space)
        {
            if (_context.Spaces.TryGetValue(space, out var anchor))
            {
                _context.Spaces.Remove(space);
                return Result.Success;
            }

            return Result.Failure;
        }
    }

    protected class Plugin72 : OVRP_1_72_0_TEST
    {
        private readonly AnchorTestFixture _context;

        private readonly Dictionary<ulong, UInt64> _eraseQueries =
            new Dictionary<ulong, UInt64>();

        private readonly Dictionary<ulong, ComponentStatusChangeRequest> _componentStatusChangeRequests =
            new Dictionary<ulong, ComponentStatusChangeRequest>();

        private readonly Dictionary<ulong, SpatialAnchorCreateInfo> _spatialAnchorCreationRequests =
            new Dictionary<ulong, SpatialAnchorCreateInfo>();

        private readonly Dictionary<ulong, HashSet<Space>> _spaceContainers = new Dictionary<ulong, HashSet<Space>>();

        // https://registry.khronos.org/OpenXR/specs/1.0/man/html/XrResult.html
        private const int XR_ERROR_SPACE_COMPONENT_STATUS_ALREADY_SET_FB = -1000113003;

        private const int XR_ERROR_SPACE_COMPONENT_STATUS_PENDING_FB = -1000113002;

        public Plugin72(AnchorTestFixture context) => _context = context;

        public IReadOnlyDictionary<ulong, UInt64> EraseQueries => _eraseQueries;

        public Dictionary<ulong, HashSet<Space>> SpaceContainers => _spaceContainers;

        public IReadOnlyDictionary<ulong, ComponentStatusChangeRequest> ComponentStatusChangeRequests
            => _componentStatusChangeRequests;

        public IReadOnlyDictionary<ulong, SpatialAnchorCreateInfo> SpatialAnchorCreationRequests
            => _spatialAnchorCreationRequests;

        public readonly Dictionary<ulong, SceneCaptureRequest> SceneCaptureRequests = new();

        public bool CreateSpatialAnchorSucceed { get; set; } = true;
        public bool SetComponentStatusSucceed { get; set; } = true;
        public bool EraseSpaceSucceed { get; set; } = true;

        public void HandleSceneCaptureRequest(OVRDeserialize.SceneCaptureCompleteData data)
        {
            if (!SceneCaptureRequests.Remove(data.RequestId, out var request))
                throw new ArgumentException($"No request with {data.RequestId}.", nameof(data));

            _context.EnqueueEvent(OVRPlugin.EventType.SceneCaptureComplete, data);
        }

        public void HandleComponentStatusChangeRequest(
            OVRDeserialize.SpaceSetComponentStatusCompleteData data)
        {
            if (!_componentStatusChangeRequests.TryGetValue(data.RequestId, out var request))
                throw new ArgumentException($"No component status change request with id {data.RequestId}",
                    nameof(data));

            if (data.Result == 0)
            {
                foreach (var (_, space) in _context.Spaces.Select(Extensions.Deconstruct))
                {
                    if (space.Handle == data.Space)
                    {
                        var shouldEnable = data.Enabled != 0;
                        switch (data.ComponentType)
                        {
                            case SpaceComponentType.Locatable:
                                space.SetEnabled<Locatable>(shouldEnable);
                                break;
                            case SpaceComponentType.RoomLayout:
                                space.SetEnabled<RoomLayout>(shouldEnable);
                                break;
                            case SpaceComponentType.SemanticLabels:
                                space.SetEnabled<SemanticLabels>(shouldEnable);
                                break;
                            case SpaceComponentType.Sharable:
                                space.SetEnabled<Sharable>(shouldEnable);
                                break;
                            case SpaceComponentType.Storable:
                                space.SetEnabled<Storable>(shouldEnable);
                                break;
                            case SpaceComponentType.Bounded2D:
                                space.SetEnabled<Bounded2D>(shouldEnable);
                                break;
                            case SpaceComponentType.Bounded3D:
                                space.SetEnabled<Bounded3D>(shouldEnable);
                                break;
                        }
                    }
                }
            }

            _componentStatusChangeRequests.Remove(data.RequestId);
            _context.EnqueueEvent(OVRPlugin.EventType.SpaceSetComponentStatusComplete, data);
        }

        public void HandleSpatialAnchorCreationRequest(
            OVRDeserialize.SpatialAnchorCreateCompleteData data, Space space)
        {
            if (!_spatialAnchorCreationRequests.TryGetValue(data.RequestId, out var request))
                throw new ArgumentException($"No spatial anchor creation request with id {data.RequestId}",
                    nameof(data));

            _spatialAnchorCreationRequests.Remove(data.RequestId);

            // Succeed so add space
            if (data.Result == 0)
            {
                _context.AddSpace(space);
            }

            _context.EnqueueEvent(OVRPlugin.EventType.SpatialAnchorCreateComplete, data);
        }

        public override Result ovrp_SetSpaceComponentStatus(ref UInt64 spaceHandle,
            SpaceComponentType componentType, Bool enable, double timeout,
            out UInt64 requestId)
        {
            if (!_context.Spaces.TryGetValue(spaceHandle, out var space))
            {
                requestId = default;
                return Result.Failure_InvalidParameter;
            }

            if (enable == Bool.True && space.HasComponent(componentType))
            {
                requestId = default;
                return (Result)XR_ERROR_SPACE_COMPONENT_STATUS_ALREADY_SET_FB;
            }

            if (_componentStatusChangeRequests.Values
                .Any(request => request.Space == space && request.ComponentType == componentType))
            {
                requestId = default;
                return (Result)XR_ERROR_SPACE_COMPONENT_STATUS_PENDING_FB;
            }

            if (!SetComponentStatusSucceed)
            {
                requestId = default;
                return OVRPlugin.Result.Failure;
            }

            requestId = _context.GenerateRequestId();
            _componentStatusChangeRequests.Add(requestId, new ComponentStatusChangeRequest
            {
                ComponentType = componentType,
                Enable = enable,
                Space = space,
                Timeout = timeout,
            });

            return Result.Success;
        }

        public override Result ovrp_GetSpaceComponentStatus(ref ulong handle,
            SpaceComponentType componentType, out Bool enabled,
            out Bool changePending)
        {
            if (!_context.Spaces.TryGetValue(handle, out var space))
            {
                enabled = default;
                changePending = default;
                return Result.Failure;
            }

            if (!space.SupportsComponent(componentType))
            {
                enabled = default;
                changePending = default;
                return Result.Failure_Unsupported;
            }

            enabled = space.HasComponent(componentType) ? Bool.True : Bool.False;
            changePending = _componentStatusChangeRequests.Values
                .Any(request => request.Space == space && request.ComponentType == componentType)
                ? Bool.True
                : Bool.False;

            return Result.Success;
        }

        public override Result ovrp_GetSpaceBoundingBox2D(ref ulong space, out Rectf rect)
        {
            if (_context.Spaces.TryGetValue(space, out var anchor) &&
                anchor.TryGetComponent<Bounded2D>(out var component))
            {
                rect = component.BoundingBox;
                return Result.Success;
            }

            rect = default;
            return Result.Failure;
        }

        public override Result ovrp_GetSpaceBoundary2D(ref ulong space,
            ref PolygonalBoundary2DInternal boundary)
        {
            if (space == 0)
            {
                return Result.Failure_InvalidParameter;
            }

            if (_context.Spaces.TryGetValue(space, out var anchor) &&
                anchor.TryGetComponent<Bounded2D>(out var component))
            {
                if (boundary.vertexCapacityInput == 0)
                {
                    boundary.vertexCountOutput = component.Boundary.Length;
                    return Result.Success;
                }

                if (boundary.vertexCapacityInput < component.Boundary.Length)
                {
                    return Result.Failure_InsufficientSize;
                }

                CopyToPtr(component.Boundary, boundary.vertices);
                boundary.vertexCountOutput = component.Boundary.Length;
                return Result.Success;
            }

            return Result.Failure_InvalidParameter;
        }

        public override Result ovrp_GetSpaceBoundingBox3D(ref ulong space, out Boundsf bounds)
        {
            if (_context.Spaces.TryGetValue(space, out var anchor) &&
                anchor.TryGetComponent<Bounded3D>(out var component))
            {
                bounds = component.BoundingBox;
                return Result.Success;
            }

            bounds = default;
            return Result.Failure;
        }

        public override Result ovrp_GetSpaceRoomLayout(ref ulong space,
            ref RoomLayoutInternal roomLayoutOut)
        {
            if (!_context.Spaces.TryGetValue(space, out var anchor) ||
                !anchor.TryGetComponent<RoomLayout>(out var component))
            {
                return Result.Failure;
            }

            var roomLayout = component.ToRoomLayout();
            var wallCount = roomLayout.wallUuids?.Length ?? 0;
            if (roomLayoutOut.wallUuidCapacityInput == 0)
            {
                roomLayoutOut.wallUuidCountOutput = wallCount;
                return Result.Success;
            }

            if (roomLayoutOut.wallUuidCapacityInput < wallCount)
            {
                return Result.Failure_InsufficientSize;
            }

            roomLayoutOut.ceilingUuid = roomLayout.ceilingUuid;
            roomLayoutOut.floorUuid = roomLayout.floorUuid;
            CopyToPtr(roomLayout.wallUuids, roomLayoutOut.wallUuids);

            return Result.Success;
        }

        public override Result ovrp_GetSpaceSemanticLabels(ref ulong space,
            ref SpaceSemanticLabelInternal labelsInternal)
        {
            if (!_context.Spaces.TryGetValue(space, out var anchor) ||
                !anchor.TryGetComponent<SemanticLabels>(out var component))
            {
                return Result.Failure;
            }

            var byteCount = System.Text.Encoding.ASCII.GetByteCount(component.ToString());
            if (labelsInternal.byteCapacityInput == 0)
            {
                labelsInternal.byteCountOutput = byteCount;
                return Result.Success;
            }

            if (labelsInternal.byteCapacityInput < byteCount)
            {
                return Result.Failure_InsufficientSize;
            }

            Marshal.Copy(System.Text.Encoding.ASCII.GetBytes(component.ToString()), 0, labelsInternal.labels,
                byteCount);
            return Result.Success;
        }

        public override Result ovrp_QuerySpaces(ref SpaceQueryInfo queryInfo, out ulong requestId)
        {
            return _context.HandleQuerySpaces(ref queryInfo, out requestId);
        }

        public override Result ovrp_RetrieveSpaceQueryResults(ref ulong requestId,
            uint resultCapacityInput, ref uint resultCountOutput, IntPtr resultsBuffer)
        {
            return _context.HandleRetrieveSpaceQueryResults(ref requestId, resultCapacityInput, ref resultCountOutput, resultsBuffer);
        }

        public override Result ovrp_GetSpaceContainer(ref UInt64 space, ref SpaceContainerInternal containerInternal)
        {
            if (!_spaceContainers.TryGetValue(space, out var results))
                return Result.Failure_InvalidParameter;

            // Two-call idiom
            if (containerInternal.uuidCapacityInput == 0)
            {
                containerInternal.uuidCountOutput = results.Count;
                return Result.Success;
            }

            if (containerInternal.uuidCapacityInput < results.Count)
            {
                return Result.Failure_InsufficientSize;
            }

            Guid[] uuids = results.Select(s => s.Uuid).ToArray();
            CopyToPtr(uuids, containerInternal.uuids);

            return Result.Success;
        }

        public override Result ovrp_CreateSpatialAnchor(
            ref SpatialAnchorCreateInfo createInfo, out ulong requestId)
        {
            if (CreateSpatialAnchorSucceed)
            {
                requestId = _context.GenerateRequestId();
                _spatialAnchorCreationRequests.Add(requestId, createInfo);
                return Result.Success;
            }
            else
            {
                requestId = default;
                return Result.Failure;
            }
        }

        public override Result ovrp_EraseSpace(ref UInt64 space, SpaceStorageLocation location, out UInt64 requestId)
        {
            if (EraseSpaceSucceed)
            {
                requestId = _context.GenerateRequestId();
                _eraseQueries.Add(requestId, space);
                return Result.Success;
            }
            else
            {
                requestId = default;
                return Result.Failure;
            }
        }

        public void HandleEraseQuery(ulong requestId, OVRPlugin.Result result, Guid uuid,
            OVRPlugin.SpaceStorageLocation spaceStorageLocation)
        {
            if (!_eraseQueries.ContainsKey(requestId))
                throw new ArgumentException($"No erase query with request id {requestId}", nameof(requestId));

            _eraseQueries.Remove(requestId);
            _context.EnqueueEvent(OVRPlugin.EventType.SpaceEraseComplete, new OVRDeserialize.SpaceEraseCompleteData
            {
                RequestId = requestId,
                Result = (int)result,
                Uuid = uuid,
                Location = spaceStorageLocation
            });
        }

        public override Result ovrp_EnumerateSpaceSupportedComponents(ref UInt64 space,
            uint componentTypesCapacityInput, out uint componentTypesCountOutput, SpaceComponentType[] componentTypes)
        {
            fixed (SpaceComponentType* buffer = componentTypes)
            {
                return ovrp_EnumerateSpaceSupportedComponents(ref space, componentTypesCapacityInput,
                    out componentTypesCountOutput, buffer);
            }
        }

        public override Result ovrp_EnumerateSpaceSupportedComponents(ref UInt64 space,
            uint componentTypesCapacityInput, out uint componentTypesCountOutput, SpaceComponentType* buffer)
        {
            var tempSpace = space;
            var internalSpace = _context.Spaces
                .Where(s => s.Key == tempSpace)
                .Select(s => s.Value)
                .FirstOrDefault();

            if (internalSpace == null)
            {
                componentTypesCountOutput = 0;
                return Result.Failure_HandleInvalid;
            }

            componentTypesCountOutput = (uint)internalSpace.SupportedComponents.Count;

            if (componentTypesCapacityInput == 0)
            {
                return Result.Success;
            }

            if (componentTypesCapacityInput < (uint)internalSpace.SupportedComponents.Count)
            {
                return Result.Failure_InsufficientSize;
            }

            for (var i = 0; i < internalSpace.SupportedComponents.Count; i++)
            {
                buffer[i] = (SpaceComponentType)internalSpace.SupportedComponents[i];
            }

            return Result.Success;
        }

        public override Result ovrp_RequestSceneCapture(ref SceneCaptureRequestInternal request, out ulong requestId)
        {
            requestId = _context.GenerateRequestId();

            SceneCaptureRequests.Add(requestId, new SceneCaptureRequest
            {
                Bytes = request.request != null ? Encoding.ASCII.GetBytes(request.request) : Array.Empty<byte>(),
            });

            return Result.Success;
        }
    }

    protected class Plugin79 : OVRP_1_79_0_TEST
    {
        private readonly AnchorTestFixture _context;

        private readonly Dictionary<ulong, ulong[]> _saveSpaceListRequests = new();

        private readonly Dictionary<ulong, ShareQueryData> _shareQueries = new();

        private readonly Dictionary<ulong, ulong> _spaceUsers = new();

        public IReadOnlyDictionary<ulong, ulong[]> SaveSpaceListRequests => _saveSpaceListRequests;

        public IReadOnlyDictionary<ulong, ShareQueryData> ShareQueries => _shareQueries;

        public IReadOnlyDictionary<ulong, ulong> SpaceUsers
            => _spaceUsers;

        public Plugin79(AnchorTestFixture fixture) => _context = fixture;

        public bool SaveSpaceListSucceed { get; set; } = true;

        public bool ShareSpaceSucceed { get; set; } = true;

#if OVR_INTERNAL_CODE
        public override Result ovrp_GetPlaneTrackingSupported(out Bool planeTrackingSupported)
        {
            planeTrackingSupported = _context.PlaneTrackingSupported ? Bool.True : Bool.False;
            return Result.Success;
        }
#endif

        static unsafe T[] ToArray<T>(T* memory, int count) where T : unmanaged
        {
            var array = new T[count];
            for (var i = 0; i < count; i++)
            {
                array[i] = memory[i];
            }

            return array;
        }

        public override Result ovrp_ShareSpaces(ulong* spaces, uint numSpaces, ulong* userHandles, uint numUsers,
            out ulong requestId)
        {
            if (ShareSpaceSucceed)
            {
                requestId = _context.GenerateRequestId();

                var data = new ShareQueryData
                {
                    Spaces = ToArray(spaces, (int)numSpaces),
                    UserHandles = ToArray(userHandles, (int)numUsers),
                };

                _shareQueries.Add(requestId, data);
                return Result.Success;
            }
            else
            {
                requestId = default;
                return Result.Failure;
            }
        }


        public override Result ovrp_CreateSpaceUser(in ulong spaceUserId, out ulong spaceUserHandle)
        {
            spaceUserHandle = _context.GenerateRequestId();
            _spaceUsers[spaceUserHandle] = spaceUserId;
            return Result.Success;
        }

        public override Result ovrp_GetSpaceUserId(in ulong spaceUserHandle, out ulong spaceUserId)
        {
            spaceUserId = SpaceUsers[spaceUserHandle];
            return Result.Success;
        }

        public void HandleShareQuery(ulong requestId, Result result)
        {
            if (!_shareQueries.ContainsKey(requestId))
                throw new ArgumentException($"No share query with request id {requestId}", nameof(requestId));

            _shareQueries.Remove(requestId);
            _context.EnqueueEvent(OVRPlugin.EventType.SpaceShareResult, new OVRDeserialize.SpaceShareResultData
            {
                RequestId = requestId,
                Result = (int)result
            });
        }


        public override Result ovrp_SaveSpaceList(ulong* spaces, uint numSpaces, SpaceStorageLocation location,
            out ulong requestId)
        {
            if (SaveSpaceListSucceed)
            {
                requestId = _context.GenerateRequestId();

                var handles = new ulong[numSpaces];
                for (var i = 0; i < numSpaces; i++)
                {
                    handles[i] = spaces[i];
                }

                _saveSpaceListRequests.Add(requestId, handles);
                return Result.Success;
            }
            else
            {
                requestId = default;
                return Result.Failure;
            }
        }

        public void HandleSaveSpaceListRequest(ulong requestId, OVRPlugin.Result result)
        {
            if (!_saveSpaceListRequests.ContainsKey(requestId))
                throw new ArgumentException($"No save query with request id {requestId}", nameof(requestId));

            _saveSpaceListRequests.Remove(requestId);
            _context.EnqueueEvent(OVRPlugin.EventType.SpaceListSaveResult, new OVRDeserialize.SpaceListSaveResultData
            {
                RequestId = requestId,
                Result = (int)result
            });
        }

        public override Result ovrp_LocateSpace2(out SpaceLocationf location, in UInt64 space,
            TrackingOrigin trackingOrigin)
        {
            if (!_context.Spaces.TryGetValue(space, out var anchor) ||
                !anchor.TryGetComponent<Locatable>(out var locatable))
            {
                location = default;
                return Result.Failure;
            }

            location = locatable.SpaceLocation;
            return Result.Success;
        }
    }

#if OVR_INTERNAL_CODE // XR_META_spatial_entity_discovery
    public class DiscoveryFilterInfo
    {
        public SpaceDiscoveryFilterType Type;
    }

    public class DiscoveryFilterInfoComponents : DiscoveryFilterInfo
    {
        public SpaceComponentType Component;
    }

    public class DiscoveryFilterInfoIds : DiscoveryFilterInfo
    {
        public Guid[] Ids;
    }

    public struct DiscoveryInfo
    {
        public DiscoveryFilterInfo[] Filters;
    }
#endif

    protected class Plugin92 : OVRP_1_92_0_TEST
    {
        private readonly AnchorTestFixture _context;

#if OVR_INTERNAL_CODE // XR_META_spatial_entity_discovery
        private readonly Dictionary<ulong, DiscoveryInfo> _discoveryQueries = new();

        private readonly Dictionary<ulong, SpaceDiscoveryResult[]> _discoveryResults = new();

        public IReadOnlyDictionary<ulong, DiscoveryInfo> DiscoveryQueries => _discoveryQueries;

        public readonly Dictionary<ulong, ulong[]> SaveSpacesRequests = new();

        public struct EraseSpacesInfo
        {
            public ulong[] Spaces;
            public Guid[] Uuids;
        }

        public readonly Dictionary<ulong, EraseSpacesInfo> EraseSpacesRequests = new();
#endif

        public Plugin92(AnchorTestFixture fixture) => _context = fixture;

#if OVR_INTERNAL_CODE
        public override Result ovrp_SaveSpaces(UInt32 spaceCount, UInt64* spaces, out UInt64 requestId)
        {
            var array = new ulong[spaceCount];
            for (var i = 0; i < spaceCount; i++)
            {
                if (spaces[i] == 0)
                {
                    requestId = default;
                    return Result.Failure_HandleInvalid;
                }

                array[i] = spaces[i];
            }

            requestId = _context.GenerateRequestId();
            SaveSpacesRequests.Add(requestId, array);
            return Result.Success;
        }

        public override Result ovrp_EraseSpaces(UInt32 spaceCount, UInt64* spaces, UInt32 uuidCount,
            Guid* uuids, out UInt64 requestId)
        {
            var spaceArray = new ulong[spaceCount];
            for (var i = 0; i < spaceCount; i++)
            {
                if (spaces[i] == 0)
                {
                    requestId = default;
                    return Result.Failure_HandleInvalid;
                }

                spaceArray[i] = spaces[i];
            }

            var uuidArray = new Guid[uuidCount];
            for (var i = 0; i < uuidCount; i++)
            {
                uuidArray[i] = uuids[i];
            }

            requestId = _context.GenerateRequestId();
            EraseSpacesRequests.Add(requestId, new EraseSpacesInfo
            {
                Spaces = spaceArray,
                Uuids = uuidArray,
            });
            return Result.Success;
        }

        public void AddSpaceDiscoveryResults(ulong requestId, SpaceDiscoveryResult[] results, bool isFinal)
        {
            if (!_discoveryQueries.ContainsKey(requestId))
                throw new ArgumentException($"No discovery query with request id {requestId}");

            _discoveryResults.Add(requestId, results);

            // fire OVRManager events
            _context.EnqueueEvent(OVRPlugin.EventType.SpaceDiscoveryResultsAvailable,
                new OVRDeserialize.SpaceDiscoveryResultsData
                {
                    RequestId = requestId
                });

            if (isFinal)
            {
                _discoveryQueries.Remove(requestId);
                _context.EnqueueEvent(OVRPlugin.EventType.SpaceDiscoveryComplete,
                    new OVRDeserialize.SpaceDiscoveryCompleteData
                    {
                        RequestId = requestId,
                        Result = results != null ? (int)Result.Success : (int)Result.Failure
                    });
            }
        }

        public void HandleDiscoverSpaces(ulong requestId, SpaceDiscoveryResult[] results)
        {
            AddSpaceDiscoveryResults(requestId, results, isFinal: true);
        }

        public override Result ovrp_DiscoverSpaces(in SpaceDiscoveryInfo info, out ulong requestId)
        {
            requestId = _context.GenerateRequestId();
            var filters = new List<DiscoveryFilterInfo>();
            for (var filterIndex = 0; filterIndex < info.NumFilters; filterIndex++)
            {
                switch (info.Filters[filterIndex]->Type)
                {
                    case SpaceDiscoveryFilterType.Component:
                    {
                        var filter = (SpaceDiscoveryFilterInfoComponents*)info.Filters[filterIndex];
                        filters.Add(new DiscoveryFilterInfoComponents
                        {
                            Type = filter->Type,
                            Component = filter->Component,
                        });
                        break;
                    }
                    case SpaceDiscoveryFilterType.Ids:
                    {
                        var filter = (SpaceDiscoveryFilterInfoIds*)info.Filters[filterIndex];
                        var uuids = new Guid[filter->NumIds];
                        for (var uuidIndex = 0; uuidIndex < uuids.Length; uuidIndex++)
                        {
                            uuids[uuidIndex] = filter->Ids[uuidIndex];
                        }

                        filters.Add(new DiscoveryFilterInfoIds
                        {
                            Type = filter->Type,
                            Ids = uuids,
                        });
                        break;
                    }
                    default:
                        throw new NotSupportedException($"Unhandled filter type {info.Filters[filterIndex]->Type}");
                }
            }

            _discoveryQueries.Add(requestId, new DiscoveryInfo
            {
                Filters = filters.ToArray(),
            });

            return Result.Success;
        }

        public override Result ovrp_RetrieveSpaceDiscoveryResults(ulong requestId, ref SpaceDiscoveryResults results)
        {
            if (!_discoveryResults.TryGetValue(requestId, out var resultsArray))
            {
                return Result.Failure_InvalidParameter;
            }

            results.ResultCountOutput = (uint)resultsArray.Length;

            // Two-call idiom
            if (results.ResultCapacityInput == 0)
            {
                return Result.Success;
            }

            if (results.ResultCapacityInput < resultsArray.Length)
            {
                return Result.Failure_InsufficientSize;
            }

            CopyToPtr(resultsArray, (IntPtr)results.Results);

            _discoveryResults.Remove(requestId);

            return Result.Success;
        }
#endif
    }

    protected class Plugin83 : OVRP_1_83_0_TEST
    {
        public OVRInput.Button CurrentState { get; set; } = OVRInput.Button.None;

        public override Result ovrp_GetControllerState6(uint controllerMask,
            ref ControllerState6 controllerState)
        {
            controllerState.Buttons = (uint)OVRInput.controllers[0].ResolveToRawMask(CurrentState);
            return Result.Success;
        }
    }

    protected class Plugin86 : OVRP_1_86_0_TEST
    {
        private readonly AnchorTestFixture _context;

        private readonly Dictionary<ulong, SpaceQueryResult[]> _spaceQueryResults =
            new Dictionary<ulong, SpaceQueryResult[]>();

        private readonly Dictionary<ulong, SpaceQueryInfo2> _spaceQueries =
            new Dictionary<ulong, SpaceQueryInfo2>();

        public IReadOnlyDictionary<ulong, SpaceQueryInfo2> SpaceQueries
            => _spaceQueries;

        public Plugin86(AnchorTestFixture fixture) => _context = fixture;

        public bool SpaceQuerySucceed { get; set; } = true;

        public void HandleSpaceQuery(ulong requestId, SpaceQueryResult[] results)
        {
            if (!_spaceQueries.ContainsKey(requestId))
                throw new ArgumentException($"No space query with request id {requestId}", nameof(requestId));

            _spaceQueries.Remove(requestId);
            _context.EnqueueEvent(OVRPlugin.EventType.SpaceQueryComplete, new OVRDeserialize.SpaceQueryCompleteData
            {
                RequestId = requestId,
                Result = results != null ? (int)Result.Success : (int)Result.Failure
            });

            if (results != null)
            {
                _spaceQueryResults.Add(requestId, results);
            }
        }

        public Result HandleQuerySpaces(ref SpaceQueryInfo queryInfo, out ulong requestId)
        {
            var queryInfo2 = AnchorTestFixture.ToSpaceQueryInfo2(ref queryInfo);
            return ovrp_QuerySpaces2(ref queryInfo2, out requestId);
        }
#if OVR_INTERNAL_CODE
        public override Result ovrp_QuerySpaces2(ref SpaceQueryInfo2 queryInfo, out ulong requestId)
#else
        public Result ovrp_QuerySpaces2(ref SpaceQueryInfo2 queryInfo, out ulong requestId)
#endif
        {
            if (SpaceQuerySucceed)
            {
                SpaceComponentType[] componentTypes =
                    new SpaceComponentType[queryInfo.ComponentsInfo.Components.Length];
                queryInfo.ComponentsInfo.Components.CopyTo(componentTypes, 0);
                queryInfo.ComponentsInfo.Components = componentTypes;

                requestId = _context.GenerateRequestId();

                var cloneQueryInfo = new SpaceQueryInfo2();
                cloneQueryInfo.QueryType = queryInfo.QueryType;
                cloneQueryInfo.MaxQuerySpaces = queryInfo.MaxQuerySpaces;
                cloneQueryInfo.Timeout = queryInfo.Timeout;
                cloneQueryInfo.Location = queryInfo.Location;
                cloneQueryInfo.ActionType = queryInfo.ActionType;
                cloneQueryInfo.FilterType = queryInfo.FilterType;

                cloneQueryInfo.IdInfo = new SpaceFilterInfoIds();
                cloneQueryInfo.IdInfo.NumIds = queryInfo.IdInfo.NumIds;
                cloneQueryInfo.IdInfo.Ids = new Guid[cloneQueryInfo.IdInfo.NumIds];
                for (int i = 0; i < cloneQueryInfo.IdInfo.NumIds; i++)
                {
                    cloneQueryInfo.IdInfo.Ids[i] = queryInfo.IdInfo.Ids[i];
                }

                cloneQueryInfo.ComponentsInfo = queryInfo.ComponentsInfo;
#if OVR_INTERNAL_CODE
                cloneQueryInfo.LocalGroupInfo = queryInfo.LocalGroupInfo;
#endif
                _spaceQueries.Add(requestId, cloneQueryInfo);

                return Result.Success;
            }
            else
            {
                requestId = default;
                return Result.Failure;
            }
        }

        public Result HandleRetrieveSpaceQueryResults(ref ulong requestId,
            uint resultCapacityInput, ref uint resultCountOutput, IntPtr resultsBuffer)
        {
            if (!_spaceQueryResults.TryGetValue(requestId, out var results))
            {
                return Result.Failure_InvalidParameter;
            }

            // Two-call idiom
            if (resultCapacityInput == 0)
            {
                resultCountOutput = (uint)results.Length;
                return Result.Success;
            }

            if (resultCapacityInput < results.Length)
            {
                return Result.Failure_InsufficientSize;
            }

            CopyToPtr(results, resultsBuffer);

            // Results are destroyed once retrieved
            _spaceQueryResults.Remove(requestId);

            return Result.Success;
        }
    }

#if OVR_INTERNAL_CODE // XR_META_return_to_room
    public Guid[] ReturnToRoomUuids => _plugin93.ReturnToRoomUuids;
#endif

    protected class Plugin93 : OVRP_1_93_0_TEST
    {
        AnchorTestFixture _context;

        public Plugin93(AnchorTestFixture fixture) => _context = fixture;

#if OVR_INTERNAL_CODE // Room entity info
        public override Result ovrp_GetSpaceRoomLabel(ulong space, uint bufferCapacityInput, out uint bufferCountOutput,
            byte* buffer)
        {
            if (!_context.Spaces.TryGetValue(space, out var instance))
            {
                bufferCountOutput = default;
                return Result.Failure_HandleInvalid;
            }

            if (!instance.TryGetComponent<RoomLabel>(out var roomLabel))
            {
                bufferCountOutput = default;
                return Result.Failure_InvalidOperation;
            }

            return TwoCall(roomLabel.Value, bufferCapacityInput, out bufferCountOutput, buffer);
        }
#endif // OVR_INTERNAL_CODE

#if OVR_INTERNAL_CODE // XR_META_return_to_room
        public Guid[] ReturnToRoomUuids = Array.Empty<Guid>();

        public override Result ovrp_RequestReturnToRoom(uint roomUuidCount, Guid* uuids)
        {
            if (uuids == null && roomUuidCount > 0)
            {
                return Result.Failure_InvalidParameter;
            }

            Array.Resize(ref ReturnToRoomUuids, (int)roomUuidCount);
            for (var i = 0; i < roomUuidCount; i++)
            {
                ReturnToRoomUuids[i] = uuids[i];
            }

            return Result.Success;
        }
#endif // XR_META_return_to_room
    }
}

#endif // OVRPLUGIN_TESTING
