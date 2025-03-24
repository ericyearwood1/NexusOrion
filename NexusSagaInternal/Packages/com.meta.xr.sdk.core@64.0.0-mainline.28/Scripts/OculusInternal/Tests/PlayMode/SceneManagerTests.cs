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
using Object = UnityEngine.Object;
using Random = UnityEngine.Random;
using Result = OVRPlugin.Result;

[TestFixture]
internal class SceneManagerTestFixture : AnchorTestFixture
{
    internal OVRSceneManager SceneManager;
    protected const double CloseToZero = 1e-6;

#if OVR_INTERNAL_CODE // Room entity info
    protected string DesiredRoomLabelValue;
#endif

    protected Space CreateRoomBox(int wallCount = 4)
    {
        var room = CreateSpace(OvrSpaceFBType.room_entity);
        var layout = room.AddComponent<RoomLayout>();

        layout.Ceiling = CreateSpace2D(OVRSceneManager.Classification.Ceiling);
        layout.Floor = CreateSpace2D(OVRSceneManager.Classification.Floor);
        layout.Walls = Enumerable.Range(0, wallCount).Select(_ =>
            CreateSpace2D(OVRSceneManager.Classification.WallFace)).ToArray();

        CreateExtrasForRoom(room);
        AddLayoutToRoom(layout, room);

#if OVR_INTERNAL_CODE // Room entity info
        if (!string.IsNullOrEmpty(DesiredRoomLabelValue))
        {
            room.AddComponent<RoomLabel>().Value = DesiredRoomLabelValue;
        }
#endif
        return room;
    }

    // Mock scene model modification from guardian room capture.
    // Added existing walls
    // Removed old containers
    // Added 3 new containers.
    protected Space ChangeSceneModel(Space currentRoom)
    {
        var newRoom = CreateSpace(OvrSpaceFBType.room_entity);
        var layout = newRoom.AddComponent<RoomLayout>();
        var currentRoomLayout = currentRoom.GetComponent<RoomLayout>();

        layout.Ceiling = currentRoomLayout.Ceiling;
        layout.Floor = currentRoomLayout.Floor;
        layout.Walls = new Space[currentRoomLayout.Walls.Length];
        currentRoomLayout.Walls.CopyTo(layout.Walls, 0);
        CreateExtrasForRoom(newRoom);
        AddLayoutToRoom(layout, newRoom);

        Spaces.Remove(currentRoom.Handle);
        foreach (var container in SpaceContainers[currentRoom])
            Spaces.Remove(container.Handle);

        return newRoom;
    }

    protected void CreateExtrasForRoom(Space room)
    {
        var spaces = new HashSet<Space>
        {
            CreateSpace2D3D(OVRSceneManager.Classification.Table, true),
            CreateSpace2D3D(OVRSceneManager.Classification.Couch, true),
            CreateSpace2D3D(OVRSceneManager.Classification.Other, false),
            CreateSpace2D(OVRSceneManager.Classification.InvisibleWallFace),
        };

        if (!SpaceContainers.TryAdd(room.Handle, spaces))
            SpaceContainers[room.Handle].UnionWith(spaces);
    }

    protected void AddLayoutToRoom(RoomLayout layout, Space room)
    {
        var spaces = new HashSet<Space>()
        {
            layout.Ceiling,
            layout.Floor
        };
        spaces.UnionWith(layout.Walls);

        if (!SpaceContainers.TryAdd(room.Handle, spaces))
            SpaceContainers[room.Handle].UnionWith(spaces);
    }

    private Space CreateSpace2D(string classification)
    {
        var space = CreateSpace(OvrSpaceFBType.plane_anchor);
        space.AddComponent<Bounded2D>();
        space.AddComponent<SemanticLabels>().Labels = new[] { classification };
        space.AddComponent<Locatable>();
        return space;
    }

    protected Space CreateSpace2D3D(string classification, bool has2D)
    {
        var space = CreateSpace(OvrSpaceFBType.plane_anchor);
        space.AddComponent<Bounded3D>();
        if (has2D) space.AddComponent<Bounded2D>();
        space.AddComponent<SemanticLabels>().Labels = new[] { classification };
        return space;
    }

    protected void ApplyRoomDimensions(Space targetRoom, float offsetX = 0, float offsetZ = 0)
    {
        if (targetRoom.GetComponent<RoomLayout>().Walls.Length > 4)
        {
            throw new ArgumentException("Supports only 4 walls for now.");
        }

        var roomLayout = targetRoom.GetComponent<RoomLayout>();
        var floorBbox = roomLayout.Floor.GetComponent<Bounded2D>();
        var offsetVector = new Vector3(offsetX, 0, offsetZ);

        floorBbox.BoundingBox.Pos = new Vector2(-4, -2).ToVector2f();
        floorBbox.BoundingBox.Size.h = 4;
        floorBbox.BoundingBox.Size.w = 6;

        floorBbox.Boundary = new Vector2[4];
        floorBbox.Boundary[0] = new Vector2(-4, -2);
        floorBbox.Boundary[1] = new Vector2(-4, 2);
        floorBbox.Boundary[2] = new Vector2(2, 2);
        floorBbox.Boundary[3] = new Vector2(2, -2);

        var floorLocatable = roomLayout.Floor.GetComponent<Locatable>();
        floorLocatable.SpaceLocation.pose.Position = offsetVector.ToVector3f();
        floorLocatable.SpaceLocation.pose.Orientation = Quaternion.Euler(-90, 0, 0).ToQuatf();

        var ceilingBbox = roomLayout.Ceiling.GetComponent<Bounded2D>();
        ceilingBbox.BoundingBox.Pos = new Vector2(-4, -2).ToVector2f();
        ceilingBbox.BoundingBox.Size.h = 4;
        ceilingBbox.BoundingBox.Size.w = 6;

        ceilingBbox.Boundary = new Vector2[4];
        ceilingBbox.Boundary[0] = new Vector2(-4, -2);
        ceilingBbox.Boundary[1] = new Vector2(-4, 2);
        ceilingBbox.Boundary[2] = new Vector2(2, 2);
        ceilingBbox.Boundary[3] = new Vector2(2, -2);

        var ceilingLocatable = roomLayout.Ceiling.GetComponent<Locatable>();
        ceilingLocatable.SpaceLocation.pose.Position = (Vector3.up * 2 + offsetVector).ToVector3f();
        ceilingLocatable.SpaceLocation.pose.Orientation = Quaternion.Euler(90, 0, 0).ToQuatf();

        for (int i = 0; i < roomLayout.Walls.Length; i++)
        {
            var wall = roomLayout.Walls[i].GetComponent<Bounded2D>();
            wall.BoundingBox.Pos = new Vector2(-4, -1).ToVector2f();
            wall.BoundingBox.Size.h = 2;
            wall.BoundingBox.Size.w = i % 2 == 0 ? 6 : 4;

            wall.Boundary = new Vector2[4];
            wall.Boundary[0] = Vector2.zero;
            wall.Boundary[1] = new Vector2(-2, 1);
            wall.Boundary[2] = new Vector2(2, 1);
            wall.Boundary[3] = new Vector2(2, -1);
        }

        var wallLocatable1 = roomLayout.Walls[0].GetComponent<Locatable>();
        wallLocatable1.SpaceLocation.pose.Position = (new Vector3(0, 1, -2) + offsetVector).ToVector3f();
        wallLocatable1.SpaceLocation.pose.Orientation = Quaternion.Euler(0, 0, 0).ToQuatf();

        var wallLocatable2 = roomLayout.Walls[1].GetComponent<Locatable>();
        wallLocatable2.SpaceLocation.pose.Position = (new Vector3(-4, 1, 0) + offsetVector).ToVector3f();
        wallLocatable2.SpaceLocation.pose.Orientation = Quaternion.Euler(0, 90, 0).ToQuatf();

        var wallLocatable3 = roomLayout.Walls[2].GetComponent<Locatable>();
        wallLocatable3.SpaceLocation.pose.Position = (new Vector3(0, 1, 2) + offsetVector).ToVector3f();
        wallLocatable3.SpaceLocation.pose.Orientation = Quaternion.Euler(0, 180, 0).ToQuatf();

        var wallLocatable4 = roomLayout.Walls[3].GetComponent<Locatable>();
        wallLocatable4.SpaceLocation.pose.Position = (new Vector3(2, 1, 0) + offsetVector).ToVector3f();
        wallLocatable4.SpaceLocation.pose.Orientation = Quaternion.Euler(0, 270, 0).ToQuatf();
    }

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        // Create a simple scene with an OVRSceneManager
        CreateDefaultCameraRig();
        SceneManager = new GameObject("Scene Manager").AddComponent<OVRSceneManager>();
#if OVR_INTERNAL_CODE // Room entity info
        DesiredRoomLabelValue = null;
#endif
    }
}

internal class SceneManagerTests : SceneManagerTestFixture
{
    private List<RoomLayout> _roomLayouts = new List<RoomLayout>();
    private List<Space> _rooms = new List<Space>();

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        SceneManager.ActiveRoomsOnly = false;
        SceneManager.PlanePrefab = new GameObject("plane prefab")
            .AddComponent<OVRSceneAnchor>();
        SceneManager.VolumePrefab = new GameObject("volume prefab")
            .AddComponent<OVRSceneAnchor>();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        TearDown();

        yield return base.UnityTearDown();
    }

    internal void TearDown()
    {
        SceneManager.InitialAnchorParent = null;
        OVRTask<bool>.Clear();
        _rooms.Clear();
        _roomLayouts.Clear();
#if OVR_INTERNAL_CODE // XR_META_return_to_room
        SceneManager.RoomEntered.RemoveAllListeners();
        SceneManager.RoomExited.RemoveAllListeners();
#endif
    }

    internal IEnumerator LoadRooms(int n = 1, bool shouldAssert = true)
    {
        _rooms.Clear();
        _roomLayouts.Clear();

        for (int i = 0; i < n; i++)
            _rooms.Add(CreateRoomBox());

        yield return LoadRooms(_rooms, shouldAssert);
    }

    protected IEnumerator LoadRooms(List<Space> rooms, bool shouldAssert = true)
    {
        if (shouldAssert)
            Assert.AreEqual(0, SpaceQueries.Count);

        SceneManager.LoadSceneModel();

        // Wait for the query to come
        yield return new WaitWhile(() => SpaceQueries.Count == 0);

        // Expect exactly 1 query to get the RoomLayout
        if (shouldAssert)
            Assert.AreEqual(1, SpaceQueries.Values
                .Where(q => q.FilterType == OVRPlugin.SpaceQueryFilterType.Components)
                .Count(q => q.ComponentsInfo.Components[0] == OVRPlugin.SpaceComponentType.RoomLayout));

        HandleAllSpaceQueries();

        // If it's a new room layout we expect to get a query for the list of uuids associated with the room
        yield return null;

        // An unchanged room layout already exist.
        if (SpaceQueries.Count == 0) yield break;
        yield return null;

        var idQueries = SpaceQueries.Values
            .Where(q => q.FilterType == OVRPlugin.SpaceQueryFilterType.Ids)
            .Select(q => q.IdInfo)
            .ToArray();

        if (shouldAssert)
            Assert.AreEqual(rooms.Count, idQueries.Length);

        for (var i = 0; i < rooms.Count; i++)
        {
            var room = rooms[i];
            var roomLayout = room.GetComponent<RoomLayout>();
            _roomLayouts.Add(roomLayout);

            var idQuery = idQueries[i];
            int expectedIdCount = SpaceContainers[room.Handle].Count;

            if (shouldAssert)
                Assert.AreEqual(expectedIdCount, idQuery.NumIds);

            var uuids = new HashSet<Guid>(idQuery.Ids.Take(idQuery.NumIds));
            if (shouldAssert)
                Assert.True(roomLayout.Uuids.All(uuid => uuids.Contains(uuid)));
        }

        yield return LocalizeAnchors();
    }

    protected IEnumerator LocalizeAnchors()
    {
        // Expect exactly _rooms.Count queries by uuids from _rooms.Count rooms
        HandleAllSpaceQueries();

        // Now we expect to get each room anchor's made locatable unless they are already locatable
        yield return null;
        if (ComponentStatusChangeRequests.Count == 0) yield break;

        var requests = ComponentStatusChangeRequests
            .Select(kvp => (kvp.Key, kvp.Value))
            .ToArray();

        foreach (var (requestId, request) in requests)
        {
            Assert.True(Spaces.TryGetValue(request.Space, out var space));

            var result = new OVRDeserialize.SpaceSetComponentStatusCompleteData
            {
                RequestId = requestId,
                Result = (int)Result.Failure,
                Space = request.Space,
                Uuid = space.Uuid,
                ComponentType = request.ComponentType,
            };

            for (var i = 0; i < _rooms.Count; i++)
            {
                var room = _rooms[i];
                if ((room.GetComponent<RoomLayout>().Handles.Contains(request.Space) ||
                     SpaceContainers[room.Handle].Contains(request.Space)) &&
                    request.ComponentType == OVRPlugin.SpaceComponentType.Locatable)
                {
                    result.Result = (int)Result.Success;
                    result.Enabled = (int)OVRPlugin.Bool.True;
                }
            }

            HandleComponentStatusChangeRequest(result);
        }

        yield return null;
    }

    private void LoadBounded2dAnchors()
    {
        List<OVRAnchor> anchors = new List<OVRAnchor>();
        OVRAnchor.FetchAnchorsAsync<OVRBounded2D>(anchors).ContinueWith((success) =>
        {
            foreach (var anchor in anchors)
            {
                if (!anchor.TryGetComponent(out OVRLocatable locatableComponent))
                {
                    continue;
                }

                if (locatableComponent.IsEnabled)
                {
                    continue;
                }

                locatableComponent.SetEnabledAsync(true)
                    .ContinueWith(success => Debug.Log($"Anchor localized."));
            }
        });
    }

    private IEnumerator TestSceneLoadComplete(int n = 1)
    {
        int sceneLoadedEventCount = 0;
        SceneManager.SceneModelLoadedSuccessfully += () => sceneLoadedEventCount++;
        yield return LoadRooms(n);
        Assert.AreEqual(1, sceneLoadedEventCount);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator SceneModelLoadedSuccessfullyEventCalledOnce()
    {
        int n = 3;
        yield return TestSceneLoadComplete(n);
        Assert.AreEqual(OVRSceneRoom.SceneRoomsList.Count, n);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator NoSceneModelToLoadEventCalledWhenSceneModelNotAvailable()
    {
        SceneManager.VerboseLogging = true;

        int noSceneModelToLoadEventCount = 0;
        SceneManager.NoSceneModelToLoad += () => noSceneModelToLoadEventCount++;
        yield return SceneManager.LoadSceneModel();

        yield return new WaitWhile(() => SpaceQueries.Count == 0);
        HandleAllSpaceQueries();
        yield return null;

        Assert.AreEqual(1, noSceneModelToLoadEventCount);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator LoadingSceneModelWhenAnchorsAreAlreadyLocalized()
    {
        var room = CreateRoomBox();
        _rooms.Add(room);

        LoadBounded2dAnchors();
        yield return new WaitWhile(() => SpaceQueries.Count == 0);

        HandleAllSpaceQueries();
        yield return LocalizeAnchors();

        // No anchors should be instantiated in the scene atm.
        Assert.False(OVRSceneRoom.SceneRooms.Any());
        Assert.False(OVRSceneAnchor.SceneAnchorsList.Any());

        int sceneLoadedEventCount = 0;
        SceneManager.SceneModelLoadedSuccessfully += () => sceneLoadedEventCount++;
        yield return LoadRooms(new[] { room }.ToList());
        Assert.AreEqual(1, sceneLoadedEventCount);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator RoomsWillLoadIfPlayerPresentInsideTheRooms()
    {
        SceneManager.ActiveRoomsOnly = true;

        var pose = OVRPlugin.PoseStatef.identity;
        pose.Pose.Position = new Vector3(3f, 1.8f, 2.5f).ToVector3f();
        SetNodePose(OVRPlugin.Node.EyeCenter, pose);

        // At origin
        var room = CreateRoomBox();
        ApplyRoomDimensions(room);
        _rooms.Add(room);

        // (2,2) meters offset
        var room2 = CreateRoomBox();
        ApplyRoomDimensions(room2, 2f, 2f);
        _rooms.Add(room2);

        // At (6,4) meters
        var room3 = CreateRoomBox();
        ApplyRoomDimensions(room3, 6f, 4f);
        _rooms.Add(room3);

        SceneManager.LoadSceneModel();
        yield return new WaitWhile(() => SpaceQueries.Count == 0);

        HandleAllSpaceQueries(); // For room layout result
        yield return LocalizeAnchors();

        int sceneLoadedEventCount = 0;
        SceneManager.SceneModelLoadedSuccessfully += () => sceneLoadedEventCount++;

        HandleAllSpaceQueries(); // For uuids result
        yield return null;
        yield return LocalizeAnchors();

        // User is currently in room 2 and 3
        Assert.AreEqual(OVRSceneRoom.SceneRoomsList.Count, 2);
        Assert.AreEqual(room2.Uuid, OVRSceneRoom.SceneRoomsList[0].GetComponent<OVRSceneAnchor>().Uuid);
        Assert.AreEqual(room3.Uuid, OVRSceneRoom.SceneRoomsList[1].GetComponent<OVRSceneAnchor>().Uuid);
        Assert.AreEqual(1, sceneLoadedEventCount);

        // reset
        SetNodePose(OVRPlugin.Node.EyeCenter, OVRPlugin.PoseStatef.identity);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator NoSceneModelToLoadIsFiredOnceIfPlayerIsNotInsideAnyRoom()
    {
        SceneManager.ActiveRoomsOnly = true;

        var playerPose = OVRPlugin.PoseStatef.identity;
        playerPose.Pose.Position = (Vector3.one * 100).ToVector3f();
        SetNodePose(OVRPlugin.Node.EyeCenter, playerPose);

        // At origin
        var room = CreateRoomBox();
        ApplyRoomDimensions(room);
        _rooms.Add(room);

        // (2,2) meters offset
        var room2 = CreateRoomBox();
        ApplyRoomDimensions(room2, 2f, 2f);
        _rooms.Add(room2);

        // At (6,4) meters
        var room3 = CreateRoomBox();
        ApplyRoomDimensions(room3, 6f, 4f);
        _rooms.Add(room3);

        SceneManager.LoadSceneModel();
        yield return new WaitWhile(() => SpaceQueries.Count == 0);

        HandleAllSpaceQueries(); // For room layout result
        yield return LocalizeAnchors();

        var sceneLoadedEventCount = 0;
        SceneManager.SceneModelLoadedSuccessfully += () => sceneLoadedEventCount++;

        var noSceneModelToLoadEventCount = 0;
        SceneManager.NoSceneModelToLoad += () => noSceneModelToLoadEventCount++;

        HandleAllSpaceQueries(); // For uuids result
        yield return null;
        yield return LocalizeAnchors();

        // User is not in any room
        Assert.AreEqual(0, sceneLoadedEventCount);
        Assert.AreEqual(1, noSceneModelToLoadEventCount);

        // reset
        SetNodePose(OVRPlugin.Node.EyeCenter, OVRPlugin.PoseStatef.identity);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator SceneAnchorsHierarchyTests()
    {
        yield return TestSceneLoadComplete();

        Assert.True(OVRSceneRoom.SceneRooms.Any());
        Assert.True(OVRSceneRoom.SceneRoomsList.All(room => room.transform.parent == null));

        Assert.True(OVRSceneAnchor.SceneAnchors.Values
            .Where(anchor => !anchor.GetComponent<OVRSceneRoom>())
            .All(anchor => anchor.transform.parent.GetComponent<OVRSceneRoom>()));

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator SceneModelReloadTest()
    {
        int sceneLoadedEventCount = 0;
        SceneManager.SceneModelLoadedSuccessfully += () => sceneLoadedEventCount++;
        yield return LoadRooms();
        Assert.AreEqual(1, sceneLoadedEventCount);
        yield return null;

        int numberOfSpawnedAnchors = OVRSceneAnchor.SceneAnchors.Count;

        // Reload scene model
        yield return LoadRooms();
        Assert.AreEqual(2, sceneLoadedEventCount);
        Assert.AreEqual(numberOfSpawnedAnchors, OVRSceneAnchor.SceneAnchors.Count);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator InitialAnchorParentTests()
    {
        var parent = new GameObject("NewParent").transform;
        SceneManager.InitialAnchorParent = parent;

        yield return TestSceneLoadComplete();

        Assert.True(OVRSceneRoom.SceneRooms.Any());
        Assert.True(OVRSceneRoom.SceneRoomsList.All(room => room.transform.parent == parent));

        Assert.True(OVRSceneAnchor.SceneAnchors.Values
            .Where(anchor => !anchor.GetComponent<OVRSceneRoom>())
            .All(anchor => anchor.transform.parent != parent));

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator RoomLayoutInformationTest()
    {
        yield return TestSceneLoadComplete();
        Assert.True(OVRSceneRoom.SceneRooms.Any());

        foreach (var sceneRoom in OVRSceneRoom.SceneRoomsList)
        {
            Assert.NotNull(sceneRoom.Floor);
            Assert.True(sceneRoom.Floor.transform.parent == sceneRoom.transform);

            Assert.NotNull(sceneRoom.Ceiling);
            Assert.True(sceneRoom.Ceiling.transform.parent == sceneRoom.transform);

            Assert.IsNotEmpty(sceneRoom.Walls);
            Assert.True(sceneRoom.Walls.All(wall => wall.transform.parent == sceneRoom.transform));
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator NewSceneModelAvailableCalledOnceOnAppResumeWhenSceneModelChanged()
    {
        int newSceneModelAvailableCount = 0;
        SceneManager.NewSceneModelAvailable += () => newSceneModelAvailableCount++;

        // Ensure no update before loading scene
        SceneManager.OnApplicationPause(false);
        Assert.AreEqual(0, newSceneModelAvailableCount);

        yield return TestSceneLoadComplete();

        // No query is pending
        Assert.AreEqual(0, SpaceQueries.Count);
        // Scene model unchanged on resume
        SceneManager.OnApplicationPause(false);

        // Wait for the scene model check query to come
        yield return new WaitWhile(() => SpaceQueries.Count == 0);
        HandleAllSpaceQueries();

        // Wait for the existing anchors query
        yield return new WaitWhile(() => SpaceQueries.Count == 0);
        HandleAllSpaceQueries();

        // Expect no scene model update.
        Assert.True(newSceneModelAvailableCount == 0);
        yield return null;

        // Change scene model
        ChangeSceneModel(_rooms[0]);

        // No query is pending
        Assert.AreEqual(0, SpaceQueries.Count);
        SceneManager.OnApplicationPause(false);

        // Wait for the query to come
        yield return new WaitWhile(() => SpaceQueries.Count == 0);

        // Expect exactly 1 query to get the RoomLayout on resume.
        Assert.AreEqual(1, SpaceQueries.Values
            .Where(q => q.FilterType == OVRPlugin.SpaceQueryFilterType.Components)
            .Count(q => q.ComponentsInfo.Components[0] == OVRPlugin.SpaceComponentType.RoomLayout));

        HandleAllSpaceQueries();
        yield return null;

        // New update available event call
        Assert.True(newSceneModelAvailableCount == 1);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator AnchorsTransformsUpdatedOnApplicationResume()
    {
        yield return TestSceneLoadComplete();

        int newSceneModelAvailableCount = 0;
        SceneManager.NewSceneModelAvailable += () => newSceneModelAvailableCount++;

        // No query is pending
        Assert.AreEqual(0, SpaceQueries.Count);

        // Change existing anchors dimension and position
        float scale = 10f;
        var randomPosition = Random.insideUnitSphere * scale;
        var randomDimension = Random.insideUnitSphere * scale;

        foreach (var space in Spaces.Values)
        {
            // we will change both the dimensions of OTHER and COUCH
            // which will test a 2D&3D object and a 3D-only object
            if (!space.TryGetComponent(out SemanticLabels semanticLabels))
                continue;

            var labels = semanticLabels.Labels;
            if (labels.Contains(OVRSceneManager.Classification.Table))
            {
#pragma warning disable CS0618 // Type or member is obsolete
                // these are the labels before going to OVRPlugin,
                // therefore we should not have DESK
                Assert.False(labels.Contains(OVRSceneManager.Classification.Desk));
#pragma warning restore CS0618 // Type or member is obsolete
                continue;
            }

            if (labels.Contains(OVRSceneManager.Classification.InvisibleWallFace))
            {
                // these are the labels before going to OVRPlugin,
                // therefore we should not have WALL_FACE
                Assert.False(labels.Contains(OVRSceneManager.Classification.WallFace));
            }

            var isOther = labels.Contains(OVRSceneManager.Classification.Other);
            var isCouch = labels.Contains(OVRSceneManager.Classification.Couch);
            if (!isOther && !isCouch)
                continue;

            var locatable = space.GetComponent<Locatable>();
            Assert.True(locatable.SpaceLocation.pose.Position.Equals(OVRPlugin.Vector3f.zero));
            locatable.SpaceLocation.pose.Position = randomPosition.ToVector3f();

            var bounded3d = space.GetComponent<Bounded3D>();
            Assert.True(bounded3d.BoundingBox.Size.Equals(OVRPlugin.Size3f.zero));
            bounded3d.BoundingBox.Size = randomDimension.ToSize3f();

            // couch also has a 2d component
            if (isCouch)
            {
                var bounded2d = space.GetComponent<Bounded2D>();
                Assert.True(bounded2d.BoundingBox.Size.Equals(OVRPlugin.Sizef.zero));
                bounded2d.BoundingBox.Size = new OVRPlugin.Sizef
                {
                    w = randomDimension.x,
                    h = randomDimension.y
                };
            }
        }

        // Scene model unchanged on resume
        SceneManager.OnApplicationPause(false);

        // Wait for the scene model check query to come
        yield return new WaitWhile(() => SpaceQueries.Count == 0);
        HandleAllSpaceQueries();

        // Wait for the existing anchors query
        yield return new WaitWhile(() => SpaceQueries.Count == 0);
        HandleAllSpaceQueries();

        // Expect no scene model update.
        Assert.True(newSceneModelAvailableCount == 0);
        yield return null;

        foreach (var sceneAnchor in OVRSceneAnchor.SceneAnchorsList)
        {
            if (!sceneAnchor.TryGetComponent(out OVRSemanticClassification classification))
                continue;

            var isTable = classification.Contains(OVRSceneManager.Classification.Table);
            if (isTable)
            {
#pragma warning disable CS0618 // Type or member is obsolete
                // unlike above, now we expect both TABLE and DESK to be labels
                // as the OVRSemanticClassification will add DESK if TABLE exists
                // (needed as long as we have deprecated support for DESK)
                Assert.True(classification.Contains(OVRSceneManager.Classification.Desk));
#pragma warning restore CS0618 // Type or member is obsolete
                continue;
            }

            if (classification.Contains(OVRSceneManager.Classification.InvisibleWallFace))
            {
                // unlike above, now we expect both WALL_OPENING and WALL_FACE to be labels
                // as the OVRSemanticClassification will add WALL_FACE if WALL_OPENING exists
                Assert.True(classification.Contains(OVRSceneManager.Classification.WallFace));
            }

            var isOther = classification.Contains(OVRSceneManager.Classification.Other);
            var isCouch = classification.Contains(OVRSceneManager.Classification.Couch);

            if (!isOther && !isCouch)
                continue;

            // Expect that we have a volume component
            Assert.True(sceneAnchor.TryGetComponent(out OVRSceneVolume volume));

            // Expect supplied position transformed to Unity coordinate space
            Assert.True(sceneAnchor.transform.position ==
                        randomPosition.ToFlippedZVector3f().FromVector3f());

            // Expect updated volume dimensions
            Assert.True(Math.Abs(volume.Width - randomDimension.x) < CloseToZero &&
                        Math.Abs(volume.Height - randomDimension.y) < CloseToZero &&
                        Math.Abs(volume.Depth - randomDimension.z) < CloseToZero);

            if (isCouch)
            {
                // Expect that we also have a plane component for couch
                Assert.True(sceneAnchor.TryGetComponent(out OVRScenePlane plane));

                // Expect updated plane dimensions
                Assert.True(Math.Abs(plane.Width - randomDimension.x) < CloseToZero &&
                            Math.Abs(plane.Height - randomDimension.y) < CloseToZero);
            }
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator SceneAnchorsParentedToAnchorParentAreInCorrectWorldSpacePose()
    {
        SceneManager.PlanePrefab = new GameObject("plane prefab")
            .AddComponent<OVRSceneAnchor>();
        var parent = new GameObject("parent").GetComponent<Transform>();

        float radius = 10f;
        Random.InitState(0);
        parent.position = Random.insideUnitSphere * radius;
        parent.rotation = Random.rotationUniform;

        SceneManager.InitialAnchorParent = parent;
        yield return LoadRooms();

        var anchors = new List<OVRSceneAnchor>();
        OVRSceneAnchor.GetSceneAnchors(anchors);

        foreach (var anchor in anchors)
        {
            if (anchor.IsComponentEnabled(OVRPlugin.SpaceComponentType.RoomLayout))
            {
                Assert.False(anchor.IsTracked);
                continue;
            }
            Assert.Less(anchor.transform.position.sqrMagnitude, CloseToZero);

            // For scene anchors, identity transforms are X left, Y up, Z backwards
            var t = anchor.transform;
            Assert.True(t.up == Vector3.up);
            Assert.True(t.forward == Vector3.back);
            Assert.True(t.right == Vector3.left);
        }
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator InstantiateSceneAnchorPrefabsOverrides()
    {
        const string prefabName = "newPlanePrefab";
        OVRSceneAnchor newPlane = new GameObject(prefabName).AddComponent<OVRSceneAnchor>();
        var prefabOverride = new OVRScenePrefabOverride()
        {
            Prefab = newPlane,
            ClassificationLabel = OVRSceneManager.Classification.Floor
        };
        SceneManager.PrefabOverrides.Add(prefabOverride);

        using var _ = new OVRObjectPool.ListScope<OVRSceneAnchor>(
            out var sceneAnchors);

        yield return LoadRooms();
        OVRSceneAnchor.GetSceneAnchors(sceneAnchors);
        Assert.AreEqual(1, OVRSceneRoom.SceneRooms.Count);

        bool newPlaneFound = false;

        foreach (var anchor in sceneAnchors)
        {
            if (anchor.gameObject.name.StartsWith(prefabName))
            {
                newPlaneFound = true;
                break;
            }
        }

        Assert.True(newPlaneFound);

        // test that we can provide null overrides
        prefabOverride.Prefab = null; // no floors
        yield return LoadRooms();
        OVRSceneAnchor.GetSceneAnchors(sceneAnchors);

        foreach (var anchor in sceneAnchors)
        {
            if (!anchor.TryGetComponent(out OVRSemanticClassification labels))
                continue;
            Assert.IsFalse(labels.Contains(OVRSceneManager.Classification.Floor));
        }

        // test that we can have null PlanePrefab
        SceneManager.PlanePrefab = null;
        SceneManager.PrefabOverrides.Clear();
        yield return LoadRooms();
        OVRSceneAnchor.GetSceneAnchors(sceneAnchors);

        foreach (var anchor in sceneAnchors)
        {
            var hasPlane = anchor.TryGetComponent(out OVRScenePlane _);
            var hasVolume = anchor.TryGetComponent(out OVRSceneVolume _);

            // either the anchor does not have a plane, or it has both a
            // plane and volume. This is because PlanePrefab only kicks in
            // when the anchor only has a plane
            Assert.IsTrue(!hasPlane || (hasPlane && hasVolume));
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanLoadMultipleAnchorQueries()
    {
        _rooms.Add(CreateRoomBox());

        var tasks = 3;
        var callbackChecks = new bool[tasks];

        for (int i = 0; i < tasks; i++)
        {
            var index = i;
            var anchors = new List<OVRAnchor>();
            OVRAnchor.FetchAnchorsAsync<OVRRoomLayout>(anchors)
                .ContinueWith((r, a) =>
                {
                    Assert.AreEqual(1, a.Count, $"More anchors than expected in task {i}.");
                    callbackChecks[index] = true;
                }, anchors);
        }

        yield return new WaitWhile(() => SpaceQueries.Count == 0);
        HandleAllSpaceQueries();

        yield return null;
        HandleAllSpaceQueries();

        for (int i = 0; i < tasks; i++)
        {
            Assert.IsTrue(callbackChecks[i], $"Task {i} callback was not called.");
        }
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator MultipleRoomLoadsOnApplicationResumeWork()
    {
        var successCalls = 0;
        SceneManager.SceneModelLoadedSuccessfully += () => successCalls++;

        SceneManager.OnApplicationPause(false);
        yield return LoadRooms(1, false);

        SceneManager.OnApplicationPause(false);
        yield return LoadRooms(1, false);

        Assert.AreEqual(2, successCalls);
    }

#if OVR_INTERNAL_CODE // Room entity info
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanAccessRoomLabelFromOVRSceneRoom()
    {
        DesiredRoomLabelValue = "تسمية الغرفة";
        yield return LoadRooms(1);
        Assert.That(Object.FindAnyObjectByType<OVRSceneRoom>().Label, Is.EqualTo(DesiredRoomLabelValue));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator MissingRoomLabelReturnsNullFromOVRSceneRoom()
    {
        DesiredRoomLabelValue = null;
        yield return LoadRooms(1);
        Assert.That(Object.FindAnyObjectByType<OVRSceneRoom>().Label, Is.Null);
    }
#endif

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator DoesRoomSetupExistWithNoRoom()
    {
        yield return AssertDoesRoomSetupExist(new[] { OVRSceneManager.Classification.Table }, false);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator DoesRoomSetupExistWithNoLoadedScene()
    {
        var room = CreateRoomBox();
        _rooms.Add(room);

        yield return AssertDoesRoomSetupExist(new string[0], true);
        yield return AssertDoesRoomSetupExist(new[] { OVRSceneManager.Classification.Table }, true);
        yield return AssertDoesRoomSetupExist(new[] { OVRSceneManager.Classification.Table, OVRSceneManager.Classification.Couch }, true);
        yield return AssertDoesRoomSetupExist(new[] { OVRSceneManager.Classification.Table, OVRSceneManager.Classification.Table }, false);
        yield return AssertDoesRoomSetupExist(new[] { OVRSceneManager.Classification.Bed }, false);

        SpaceContainers[room.Handle].Add(CreateSpace2D3D(OVRSceneManager.Classification.Table, true));
        yield return AssertDoesRoomSetupExist(new[] { OVRSceneManager.Classification.Table, OVRSceneManager.Classification.Table }, true);
    }

    private IEnumerator AssertDoesRoomSetupExist(string[] classifications, bool expectation)
    {
        // No query is pending
        Assert.AreEqual(0, SpaceQueries.Count);

        bool wasCalled = false;
        SceneManager.DoesRoomSetupExist(classifications)
            .ContinueWith(result =>
            {
                Assert.AreEqual(expectation, result, $"{string.Join(", ", classifications)} {(expectation ? "should" : "should not")} be part of test scene setup.");
                wasCalled = true;
            });

        HandleAllSpaceQueries();

        yield return new WaitWhile(() => SpaceQueries.Count == 0);
        HandleAllSpaceQueries();

        yield return null;
        HandleAllSpaceQueries();

        Assert.True(wasCalled, $"{nameof(OVRSceneManager.DoesRoomSetupExist)} callback was not called.");
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator GetSceneAnchorsOfTypeOnlyReturnsSceneAnchorsWithThatType()
    {
        // So that we don't pick up the "prefabs" that are also part of the scene graph
        SceneManager.InitialAnchorParent = new GameObject("Scene Root").transform;

        yield return LoadRooms(1);

        CheckType<OVRSceneAnchor>();
        CheckType<OVRScenePlane>();
        CheckType<OVRSceneVolume>();
        CheckType<OVRSceneRoom>();
        CheckType<OVRScenePlaneMeshFilter>();
        CheckType<OVRSceneVolumeMeshFilter>();

        void CheckType<T>() where T : Object
        {
            var sceneAnchors = new List<T>();
            OVRSceneAnchor.GetSceneAnchorsOfType<T>(sceneAnchors);
            Assert.That(SceneManager.InitialAnchorParent.GetComponentsInChildren<T>(), Is.EquivalentTo(sceneAnchors),
                $"Failed for {typeof(T).Name}");
        }
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator SceneModelTelemetryTest()
    {
        // this only goes through OVRSceneManager
        yield return LoadRooms(1, false);
        OVRTelemetry.Expect(OVRTelemetryConstants.Scene.MarkerId.UseOVRSceneManager)
            .AddAnnotation(OVRTelemetryConstants.Scene.AnnotationType.UsingBasicPrefabs, "true")
            .AddAnnotation(OVRTelemetryConstants.Scene.AnnotationType.UsingPrefabOverrides, "false")
            .AddAnnotation(OVRTelemetryConstants.Scene.AnnotationType.ActiveRoomsOnly, "false");
        OVRTelemetry.TestExpectations();
        OVRTelemetry.ResetExpectations();

        // reset SceneManager and set properties before Start()
        GameObject.Destroy(SceneManager.gameObject);
        yield return null; // to avoid 2 OVRSceneManagers
        SceneManager = new GameObject("Scene Manager").AddComponent<OVRSceneManager>();
        SceneManager.ActiveRoomsOnly = true;

        // check active rooms and model loader telemetry
        yield return LoadRooms(1, false);
        OVRTelemetry.Expect(OVRTelemetryConstants.Scene.MarkerId.UseOVRSceneManager)
            .AddAnnotation(OVRTelemetryConstants.Scene.AnnotationType.ActiveRoomsOnly, "true");
        OVRTelemetry.TestExpectations();
    }

#if OVR_INTERNAL_CODE // XR_META_return_to_room
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator ReturnToRoomTelemetryTest()
    {
        // merge into SceneModelTelemetryTest when
        // XR_META_return_to_room is no longer internal/partner
        yield return LoadRooms(1, false);
        OVRTelemetry.Expect(OVRTelemetryConstants.Scene.MarkerId.UseOVRSceneManager)
            .AddAnnotation(OVRTelemetryConstants.Scene.AnnotationType.ReturnToRooms, "false");
        OVRTelemetry.TestExpectations();
        OVRTelemetry.ResetExpectations();

        // reset SceneManager and set properties before Start()
        GameObject.Destroy(SceneManager.gameObject);
        yield return null; // to avoid 2 OVRSceneManagers
        SceneManager = new GameObject("Scene Manager").AddComponent<OVRSceneManager>();
        SceneManager.KeepUserInRooms = true;

        // check return to rooms telemetry
        yield return LoadRooms(1, false);
        OVRTelemetry.Expect(OVRTelemetryConstants.Scene.MarkerId.UseOVRSceneManager)
            .AddAnnotation(OVRTelemetryConstants.Scene.AnnotationType.ReturnToRooms, "true");
        OVRTelemetry.TestExpectations();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator ReturnToRoomEventFired()
    {
        yield return LoadRooms(3);
        var space = _rooms[0];
        var sceneRooms = UnityEngine.Object.FindObjectsByType<OVRSceneRoom>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        var expectedUuid = space.Uuid;
        var expectedSceneRoom = sceneRooms.First(r => r.GetComponent<OVRSceneAnchor>().Uuid == expectedUuid);

        var uuidEntered = Guid.Empty;
        var onEntered = new Action<Guid>(uuid => uuidEntered = uuid);
        OVRAnchor.RoomEntered += onEntered;

        var uuidExited = Guid.Empty;
        var onExited = new Action<Guid>(uuid => uuidExited = uuid);
        OVRAnchor.RoomExited += onExited;

        OVRSceneRoom roomEntered = null;
        SceneManager.RoomEntered.AddListener(sceneRoom => roomEntered = sceneRoom);

        OVRSceneRoom roomExited = null;
        SceneManager.RoomExited.AddListener(sceneRoom => roomExited = sceneRoom);

        EnqueueEvent(OVRPlugin.EventType.ReturnToRoomUserLocationChanged, new OVRDeserialize.ReturnToRoomUserLocationChanged
        {
            RoomUuid = expectedUuid,
            UserLocation = OVRPlugin.ReturnToRoomUserLocation.Outside,
        });

        // Wait a frame for the event to be handled by the OVRSceneManager
        yield return null;

        Assert.That(roomExited, Is.EqualTo(expectedSceneRoom));
        Assert.That(uuidExited, Is.EqualTo(expectedUuid));

        // Move back inside
        EnqueueEvent(OVRPlugin.EventType.ReturnToRoomUserLocationChanged, new OVRDeserialize.ReturnToRoomUserLocationChanged
        {
            RoomUuid = expectedUuid,
            UserLocation = OVRPlugin.ReturnToRoomUserLocation.Inside,
        });

        // Wait a frame for the event to be handled by the OVRSceneManager
        yield return null;

        Assert.That(roomEntered, Is.EqualTo(expectedSceneRoom));
        Assert.That(uuidEntered, Is.EqualTo(expectedUuid));

        OVRAnchor.RoomEntered -= onEntered;
        OVRAnchor.RoomExited -= onExited;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator RequestReturnToRoomFromOVRSceneRoomCallsOpenXRFunctionWithCorrectUuids()
    {
        yield return LoadRooms(10);
        var rooms = UnityEngine.Object.FindObjectsByType<OVRSceneRoom>(FindObjectsSortMode.None);
        Assert.That(rooms.Length, Is.EqualTo(10));

        var roomsToReturnTo = new [] { 1, 5, 9 };

        foreach (var index in roomsToReturnTo)
        {
            rooms[index].ShouldAttemptToKeepUserInRoom = true;
        }

        yield return null;

        var expectedUuids = rooms
            .Where((_, index) => roomsToReturnTo.Contains(index))
            .Select(room => room.GetComponent<OVRSceneAnchor>().Uuid)
            .ToHashSet();

        Assert.That(ReturnToRoomUuids.Length, Is.EqualTo(expectedUuids.Count));
        Assert.That(ReturnToRoomUuids.ToHashSet(), Is.EquivalentTo(expectedUuids));

        // Make sure we can remove one
        rooms[5].ShouldAttemptToKeepUserInRoom = false;
        expectedUuids.Remove(rooms[5].GetComponent<OVRSceneAnchor>().Uuid);

        yield return null;

        Assert.That(ReturnToRoomUuids.Length, Is.EqualTo(expectedUuids.Count));
        Assert.That(ReturnToRoomUuids.ToHashSet(), Is.EquivalentTo(expectedUuids));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator RequestReturnToRoomIsInvokedWhenARoomIsDestroyed()
    {
        yield return LoadRooms(5);
        var rooms = UnityEngine.Object.FindObjectsByType<OVRSceneRoom>(FindObjectsSortMode.None);

        foreach (var room in rooms)
        {
            Assert.That(room.ShouldAttemptToKeepUserInRoom, Is.False);
            room.ShouldAttemptToKeepUserInRoom = true;
        }

        yield return null;

        Assert.That(ReturnToRoomUuids.Length, Is.EqualTo(rooms.Length));

        // Now destroy one
        var removedUuid = rooms[3].GetComponent<OVRSceneAnchor>().Uuid;
        UnityEngine.Object.Destroy(rooms[3].gameObject);

        yield return null;

        Assert.That(ReturnToRoomUuids.Length, Is.EqualTo(rooms.Length - 1));
        Assert.That(ReturnToRoomUuids.Contains(removedUuid), Is.False);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator RequestReturnToRoomForAllRooms()
    {
        var sceneManager = UnityEngine.Object.FindAnyObjectByType<OVRSceneManager>();
        sceneManager.KeepUserInRooms = true;

        yield return LoadRooms(3);

        var rooms = UnityEngine.Object.FindObjectsByType<OVRSceneRoom>(FindObjectsSortMode.None);
        Assert.That(rooms.Length, Is.EqualTo(3));

        foreach (var room in rooms)
            Assert.That(room.ShouldAttemptToKeepUserInRoom, Is.True);
    }
#endif
}

#endif // OVRPLUGIN_TESTING
