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

using NUnit.Framework;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.TestTools.Utils;

internal unsafe class SceneObjectGeometryTests : OVRPluginPlayModeTest
{
    // fixture level variables
    private GameObject _camera;

    // test level variables (reset after each test)
    private readonly List<GameObject> _gameObjects = new List<GameObject>();
    private OVRAnchor TestAnchor => new OVRAnchor(new OVRSpace(1), Guid.Empty);

    [Test]
    public void ScenePlaneGeometry()
    {
        new MockBuilder()
            .AddLocate(Vector3.zero, Quaternion.identity)
            .AddPlane(new Vector2(1, 2), new Vector2(1.2f, 3.2f))
            .BuildAndSet();

        var gameObj = new GameObject("ScenePlane",
            typeof(OVRSceneAnchor),
            typeof(OVRScenePlane));
        _gameObjects.Add(gameObj);

        gameObj.GetComponent<OVRSceneAnchor>().Initialize(TestAnchor);

        Assert.That(gameObj.TryGetComponent(out OVRScenePlane plane), Is.True);
        Assert.That(gameObj.TryGetComponent(out OVRSceneVolume _), Is.False);
        Assert.That(plane.Width, Is.EqualTo(1.2f));
        Assert.That(plane.Height, Is.EqualTo(3.2f));
        Assert.That(plane.Offset, Is.EqualTo(new Vector2(-1.6f, 3.6f)));
    }

    [Test]
    public void SceneVolumeGeometry()
    {
        new MockBuilder()
            .AddLocate(Vector3.zero, Quaternion.identity)
            .AddVolume(new Vector3(-1, -1, -1), Vector3.one * 2)
            .BuildAndSet();

        var gameObj = new GameObject("SceneVolume",
            typeof(OVRSceneAnchor),
            typeof(OVRSceneVolume));
        _gameObjects.Add(gameObj);

        gameObj.GetComponent<OVRSceneAnchor>().Initialize(TestAnchor);

        Assert.That(gameObj.TryGetComponent(out OVRScenePlane _), Is.False);
        Assert.That(gameObj.TryGetComponent(out OVRSceneVolume volume), Is.True);
        Assert.That(volume.Width, Is.EqualTo(2));
        Assert.That(volume.Height, Is.EqualTo(2));
        Assert.That(volume.Depth, Is.EqualTo(2));
        Assert.That(volume.Offset, Is.EqualTo(new Vector3(0, 0, 1)));
    }

    [Test]
    public void ScenePlaneVolumeChildren()
    {
        // test for the various conditions of whether the
        // plane or the volume scaling/offset is applied
        // depending on the transform type

        new MockBuilder()
            .AddLocate(Vector3.one, Quaternion.identity)
            .AddPlane(new Vector2(1, 2), new Vector2(4, 5))
            .AddVolume(new Vector3(0, 1, -2), new Vector3(2, 2, 3))
            .BuildAndSet();

        var comparer = new Vector3EqualityComparer(1e-4f);

        // create a top-level gameobject with 3 children
        var gameObj = new GameObject("ScenePlaneVolume",
            typeof(OVRSceneAnchor),
            typeof(OVRScenePlane),
            typeof(OVRSceneVolume));
        var child1 = new GameObject("Child1").transform;
        var child2 = new GameObject("Child2").transform;
        var child3 = new GameObject("Child3").transform;
        child1.SetParent(gameObj.transform);
        child2.SetParent(gameObj.transform);
        child3.SetParent(gameObj.transform);
        Assert.That(gameObj.transform.childCount, Is.EqualTo(3));
        _gameObjects.AddRange(new[] { gameObj, child1.gameObject,
            child2.gameObject, child3.gameObject });

        var anchor = gameObj.GetComponent<OVRSceneAnchor>();
        var plane = gameObj.GetComponent<OVRScenePlane>();
        var volume = gameObj.GetComponent<OVRSceneVolume>();

        // inline helper function to reset state
        Action resetOVRObjects = () =>
        {
            // we have to recreate these because we can't
            // initialize multiple times to do the reset
            UnityEngine.Object.DestroyImmediate(volume);
            UnityEngine.Object.DestroyImmediate(plane);
            UnityEngine.Object.DestroyImmediate(anchor);
            anchor = gameObj.AddComponent<OVRSceneAnchor>();
            plane = gameObj.AddComponent<OVRScenePlane>();
            volume = gameObj.AddComponent<OVRSceneVolume>();

            child1.localPosition = child2.localPosition = child3.localPosition = Vector3.zero;
            child1.localScale = child2.localScale = child3.localScale = Vector3.one;
        };

        // 1. set plane to true, volume to false, check for plane offset
        plane.ScaleChildren = plane.OffsetChildren = true;
        volume.ScaleChildren = volume.OffsetChildren = false;
        anchor.Initialize(TestAnchor);
        Assert.That(child1.localScale, Is.EqualTo(new Vector3(4, 5, 1)));
        Assert.That(child1.localPosition, Is.EqualTo(new Vector3(-3, 4.5f, 0)).Using(comparer));
        Assert.That(child2.localScale, Is.EqualTo(child1.localScale));
        Assert.That(child2.localPosition, Is.EqualTo(child1.localPosition));
        Assert.That(child3.localScale, Is.EqualTo(child1.localScale));
        Assert.That(child3.localPosition, Is.EqualTo(child1.localPosition));

        // 2. set plane to false, volume to true, check for volume offset
        resetOVRObjects();
        plane.ScaleChildren = plane.OffsetChildren = false;
        volume.ScaleChildren = volume.OffsetChildren = true;
        anchor.Initialize(TestAnchor);
        Assert.That(child1.localScale, Is.EqualTo(new Vector3(2, 2, 3)));
        Assert.That(child1.localPosition, Is.EqualTo(new Vector3(-1, 0.5f, 3)).Using(comparer));
        Assert.That(child2.localScale, Is.EqualTo(child1.localScale));
        Assert.That(child2.localPosition, Is.EqualTo(child1.localPosition));
        Assert.That(child3.localScale, Is.EqualTo(child1.localScale));
        Assert.That(child3.localPosition, Is.EqualTo(child1.localPosition));

        // 3. set plane to true, volume to true, check for volume offset
        resetOVRObjects();
        plane.ScaleChildren = plane.OffsetChildren = true;
        volume.ScaleChildren = volume.OffsetChildren = true;
        anchor.Initialize(TestAnchor);
        Assert.That(child1.localScale, Is.EqualTo(new Vector3(2, 2, 3)));
        Assert.That(child1.localPosition, Is.EqualTo(new Vector3(-1, 0.5f, 3)).Using(comparer));
        Assert.That(child2.localScale, Is.EqualTo(child1.localScale));
        Assert.That(child2.localPosition, Is.EqualTo(child1.localPosition));
        Assert.That(child3.localScale, Is.EqualTo(child1.localScale));
        Assert.That(child3.localPosition, Is.EqualTo(child1.localPosition));

        // 4. set plane and volume to false, check for 0 offset
        resetOVRObjects();
        plane.ScaleChildren = plane.OffsetChildren = false;
        volume.ScaleChildren = volume.OffsetChildren = false;
        anchor.Initialize(TestAnchor);
        Assert.That(child1.localScale, Is.EqualTo(Vector3.one));
        Assert.That(child1.localPosition, Is.EqualTo(Vector3.zero));
        Assert.That(child2.localScale, Is.EqualTo(child1.localScale));
        Assert.That(child2.localPosition, Is.EqualTo(child1.localPosition));
        Assert.That(child3.localScale, Is.EqualTo(child1.localScale));
        Assert.That(child3.localPosition, Is.EqualTo(child1.localPosition));

        // 5. add transform types, set plane and volume to true, check for correct offset
        resetOVRObjects();
        plane.ScaleChildren = plane.OffsetChildren = true;
        volume.ScaleChildren = volume.OffsetChildren = true;
        child1.gameObject.AddComponent<OVRSceneObjectTransformType>().TransformType
            = OVRSceneObjectTransformType.Transformation.Volume;
        child2.gameObject.AddComponent<OVRSceneObjectTransformType>().TransformType
            = OVRSceneObjectTransformType.Transformation.Plane;
        child3.gameObject.AddComponent<OVRSceneObjectTransformType>().TransformType
            = OVRSceneObjectTransformType.Transformation.None;
        anchor.Initialize(TestAnchor);
        Assert.That(child1.localScale, Is.EqualTo(new Vector3(2, 2, 3)));
        Assert.That(child1.localPosition, Is.EqualTo(new Vector3(-1, 0.5f, 3)).Using(comparer));
        Assert.That(child2.localScale, Is.EqualTo(new Vector3(4, 5, 1)));
        Assert.That(child2.localPosition, Is.EqualTo(new Vector3(-3, 4.5f, 0)).Using(comparer));
        Assert.That(child3.localScale, Is.EqualTo(Vector3.one));
        Assert.That(child3.localPosition, Is.EqualTo(Vector3.zero));
    }

    [TearDown]
    public void TearDown()
    {
        OVRPlugin.OVRP_1_64_0.mockObj = new OVRPlugin.OVRP_1_64_0_TEST();
        OVRPlugin.OVRP_1_72_0.mockObj = new OVRPlugin.OVRP_1_72_0_TEST();
        OVRPlugin.OVRP_1_79_0.mockObj = new OVRPlugin.OVRP_1_79_0_TEST();

        foreach (var gameObj in _gameObjects)
            UnityEngine.Object.DestroyImmediate(gameObj);
        _gameObjects.Clear();
    }

    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        // we need the camera for OVRSceneAnchor transforms
        _camera = new GameObject("Camera", typeof(Camera))
        {
            tag = "MainCamera"
        };
    }

    [OneTimeTearDown]
    public void OneTimeTearDown()
    {
        if (_camera != null) UnityEngine.Object.DestroyImmediate(_camera);
    }

#region Mock implementations

    private class MockBuilder
    {
        public Vector3 Position = Vector3.zero;
        public Quaternion Orientation = Quaternion.identity;

        public bool Has2D = false;
        public Vector2 MinPoint2D = Vector2.one * -0.5f;
        public Vector2 Size2D = Vector2.one;

        public bool Has3D = false;
        public Vector3 MinPoint3D = Vector3.one * -0.5f;
        public Vector3 Size3D = Vector3.one;

        public MockBuilder AddLocate(Vector3 location, Quaternion rotation)
        {
            Position = location;
            Orientation = rotation;
            return this;
        }

        public MockBuilder AddPlane(Vector2 minPoint, Vector2 size)
        {
            Has2D = true;
            MinPoint2D = minPoint;
            Size2D = size;

            return this;
        }

        public MockBuilder AddVolume(Vector3 minPoint, Vector3 size)
        {
            Has3D = true;
            MinPoint3D = minPoint;
            Size3D = size;

            return this;
        }

        public void BuildAndSet()
        {
            var mock64 = new Mock64
            {
                Position = Position.ToVector3f(),
                Orientation = Orientation.ToQuatf()
            };
            var mock72 = new Mock72
            {
                Has2D = Has2D,
                Pos2 = MinPoint2D.ToVector2f(),
                Size2 = Size2D.ToSizef(),
                Has3D = Has3D,
                Pos3 = MinPoint3D.ToVector3f(),
                Size3 = Size3D.ToSize3f(),
            };
            var mock79 = new Mock79()
            {
                Position = Position.ToVector3f(),
                Orientation = Orientation.ToQuatf()
            };

            OVRPlugin.OVRP_1_64_0.mockObj = mock64;
            OVRPlugin.OVRP_1_72_0.mockObj = mock72;
            OVRPlugin.OVRP_1_79_0.mockObj = mock79;
        }
    }

    private class Mock64 : OVRPlugin.OVRP_1_64_0_TEST
    {
        public OVRPlugin.Vector3f Position;
        public OVRPlugin.Quatf Orientation;

        public override OVRPlugin.Result ovrp_LocateSpace(
            ref OVRPlugin.Posef location, ref UInt64 space,
            OVRPlugin.TrackingOrigin trackingOrigin)
        {
            location = new OVRPlugin.Posef { Orientation = Orientation, Position = Position };
            return OVRPlugin.Result.Success;
        }
    }

    private class Mock72 : OVRPlugin.OVRP_1_72_0_TEST
    {
        public bool Has2D;
        public OVRPlugin.Vector2f Pos2;
        public OVRPlugin.Sizef Size2;

        public bool Has3D;
        public OVRPlugin.Vector3f Pos3;
        public OVRPlugin.Size3f Size3;

        public override OVRPlugin.Result ovrp_EnumerateSpaceSupportedComponents(ref UInt64 space,
            uint componentTypesCapacityInput, out uint componentTypesCountOutput,
            OVRPlugin.SpaceComponentType* componentTypes)
        {
            componentTypesCountOutput = 3;

            if (componentTypesCapacityInput == 0)
            {
                return OVRPlugin.Result.Success;
            }

            if (componentTypesCapacityInput < 3)
            {
                return OVRPlugin.Result.Failure_InsufficientSize;
            }

            componentTypes[0] = OVRPlugin.SpaceComponentType.Locatable;
            componentTypes[1] = OVRPlugin.SpaceComponentType.Bounded2D;
            componentTypes[2] = OVRPlugin.SpaceComponentType.Bounded3D;

            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_EnumerateSpaceSupportedComponents(ref UInt64 space,
            uint componentTypesCapacityInput, out uint componentTypesCountOutput,
             OVRPlugin.SpaceComponentType[] componentTypes)
        {
            fixed (OVRPlugin.SpaceComponentType* buffer = componentTypes)
            {
                return ovrp_EnumerateSpaceSupportedComponents(ref space, componentTypesCapacityInput,
                    out componentTypesCountOutput, buffer);
            }
        }

        public override OVRPlugin.Result ovrp_GetSpaceBoundingBox2D(ref ulong space, out OVRPlugin.Rectf rect)
        {
            rect = new OVRPlugin.Rectf { Pos = Pos2, Size = Size2 };
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceBoundingBox3D(ref ulong space, out OVRPlugin.Boundsf bounds)
        {
            bounds = new OVRPlugin.Boundsf { Pos = Pos3, Size = Size3 };
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceComponentStatus(ref ulong space,
            OVRPlugin.SpaceComponentType componentType, out OVRPlugin.Bool enabled,
            out OVRPlugin.Bool changePending)
        {
            enabled = OVRPlugin.Bool.False;
            changePending = OVRPlugin.Bool.False;
            if ((componentType == OVRPlugin.SpaceComponentType.Bounded2D && Has2D) ||
                (componentType == OVRPlugin.SpaceComponentType.Bounded3D && Has3D))
            {
                enabled = OVRPlugin.Bool.True;
            }
            return OVRPlugin.Result.Success;
        }
    }

    private class Mock79 : OVRPlugin.OVRP_1_79_0_TEST
    {
        public OVRPlugin.Vector3f Position;
        public OVRPlugin.Quatf Orientation;
        private const OVRPlugin.SpaceLocationFlags LocationFlags =
            OVRPlugin.SpaceLocationFlags.OrientationValid | OVRPlugin.SpaceLocationFlags.PositionValid | OVRPlugin.SpaceLocationFlags.OrientationTracked | OVRPlugin.SpaceLocationFlags.PositionTracked;

        public override OVRPlugin.Result ovrp_LocateSpace2(out OVRPlugin.SpaceLocationf location, in ulong space, OVRPlugin.TrackingOrigin trackingOrigin)
        {
            location = new OVRPlugin.SpaceLocationf
            {
                pose = new OVRPlugin.Posef { Orientation = Orientation, Position = Position },
                locationFlags = LocationFlags
            };
            return OVRPlugin.Result.Success;
        }
    }

#endregion
}

#endif
