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
using System.Threading;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using UnityEngine.TestTools;
using UnityEngine;
using static OVRPlugin;
using Object = UnityEngine.Object;

internal unsafe class ScenePlaneMeshFilterTests : OVRPluginPlayModeTest
{
    private OVRSceneManager _sceneManager;

    private static int _boundaryPointCount;

    private static bool _failGetBoundaryOnSecondCall;

    private static int _lock;

    private struct LockScope : IDisposable
    {
        public LockScope(int value) => _lock = value;
        public void Dispose() => _lock = 0;
    }

    private class Mock1 : OVRP_1_0_0_TEST
    {
        public override TrackingOrigin ovrp_GetTrackingOriginType()
        {
            return TrackingOrigin.Stage;
        }
    }

    private class Mock64 : OVRP_1_64_0_TEST
    {
        public override Result ovrp_LocateSpace(ref Posef location, ref UInt64 space, TrackingOrigin trackingOrigin)
        {
            location = Posef.identity;
            return Result.Success;
        }
    }

    private class Mock65 : OVRP_1_65_0_TEST
    {
        public override Result ovrp_DestroySpace(ref ulong space) => Result.Success;
    }

    private class Mock72 : OVRP_1_72_0_TEST
    {
        public override Result ovrp_GetSpaceBoundary2D(ref UInt64 space,
            ref PolygonalBoundary2DInternal boundaryInternal)
        {
            while (_lock != 0)
            {
                Thread.Yield();
            }

            // Simulate the fact this takes some time
            Thread.Sleep(1);

            var count = _boundaryPointCount;

            if (count == 0)
            {
                return Result.Failure_NotInitialized;
            }

            if (boundaryInternal.vertexCapacityInput == 0)
            {
                boundaryInternal.vertexCountOutput = count;
                return Result.Success;
            }

            if (boundaryInternal.vertexCapacityInput != count)
            {
                return Result.Failure_InvalidParameter;
            }

            if (_failGetBoundaryOnSecondCall)
            {
                return Result.Failure_OperationFailed;
            }

            GetBoundaryVertices((Vector2*)boundaryInternal.vertices, count);
            boundaryInternal.vertexCountOutput = count;

            return Result.Success;
        }

        public override Result ovrp_GetSpaceBoundingBox2D(ref UInt64 space, out Rectf rect)
        {
            rect = new Rectf
            {
                Pos = default,
                Size = new Sizef { w = 1, h = 1 },
            };
            return Result.Success;
        }

        public override Result ovrp_GetSpaceComponentStatus(ref UInt64 space,
            SpaceComponentType componentType, out Bool enabled, out Bool changePending)
        {
            enabled = componentType switch
            {
                SpaceComponentType.Locatable => Bool.True,
                SpaceComponentType.Bounded2D => Bool.True,
                _ => Bool.False,
            };
            changePending = Bool.False;
            return Result.Success;
        }
    }

    private MeshFilter _meshFilter;

    private OVRScenePlaneMeshFilter _planeMesh;

    private OVRScenePlane _plane;

    private GameObject _gameObject;

    private GameObject _mainCamera;

    private static void GetBoundaryVertices(NativeArray<Vector2> vertices) =>
        GetBoundaryVertices((Vector2*)vertices.GetUnsafePtr(), vertices.Length);

    private static void GetBoundaryVertices(Vector2* vertices, int count)
    {
        // It doesn't matter what boundary we provide, but it needs to be a valid shape, so use a circle.
        for (var i = 0; i < count; i++)
        {
            var a = Mathf.PI * 2 * i / count;
            vertices[i] = new Vector2(Mathf.Cos(a), Mathf.Sin(a));
        }
    }

    private static OVRScenePlaneMeshFilter CreatePlane()
    {
        var go = new GameObject("Plane Mesh",
            typeof(MeshFilter),
            typeof(OVRSceneAnchor),
            typeof(OVRScenePlane),
            typeof(OVRScenePlaneMeshFilter));

        go.GetComponent<OVRSceneAnchor>().Initialize(new OVRAnchor(new OVRSpace(1), Guid.NewGuid()));
        return go.GetComponent<OVRScenePlaneMeshFilter>();
    }

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        OVRP_1_0_0.mockObj = new Mock1();
        OVRP_1_64_0.mockObj = new Mock64();
        OVRP_1_65_0.mockObj = new Mock65();
        OVRP_1_72_0.mockObj = new Mock72();

        _boundaryPointCount = 0;
        _lock = 0;
        _failGetBoundaryOnSecondCall = false;

        _mainCamera = new GameObject("Camera", typeof(Camera))
        {
            tag = "MainCamera"
        };
        _planeMesh = CreatePlane();
        _planeMesh.enabled = false;
        _plane = _planeMesh.GetComponent<OVRScenePlane>();
        _plane.enabled = false;
        _meshFilter = _planeMesh.GetComponent<MeshFilter>();
        _gameObject = _plane.gameObject;
    }

    [OneTimeTearDown]
    public void OneTimeTearDown()
    {
        if (_gameObject)
        {
            Object.DestroyImmediate(_gameObject);
        }

        if (_mainCamera)
        {
            Object.DestroyImmediate(_mainCamera);
        }

        OVRP_1_0_0.mockObj = new OVRP_1_0_0_TEST();
        OVRP_1_64_0.mockObj = new OVRP_1_64_0_TEST();
        OVRP_1_65_0.mockObj = new OVRP_1_65_0_TEST();
        OVRP_1_72_0.mockObj = new OVRP_1_72_0_TEST();
    }

    private IEnumerator GenerateMesh(int vertexCount)
    {
        if (_planeMesh.JobHandleForTesting.HasValue)
        {
            yield return new WaitWhile(() => _planeMesh.JobHandleForTesting.HasValue);
        }

        _boundaryPointCount = vertexCount;
        _plane.RequestBoundary();
        yield return new WaitUntil(() => _plane.Boundary.Count == vertexCount);
        Assert.IsTrue(_planeMesh.JobHandleForTesting.HasValue);
        yield return WaitForMeshAndVerifyResults();
    }

    private IEnumerator WaitForMeshAndVerifyResults()
    {
        yield return new WaitWhile(() =>
            _plane.Boundary.Count == 0 ||
            _planeMesh.JobHandleForTesting.HasValue);

        Assert.AreEqual(_boundaryPointCount, _plane.Boundary.Count);

        using var boundary = new NativeArray<Vector2>(_boundaryPointCount, Allocator.Temp);
        GetBoundaryVertices(boundary);

        var expectedVertices = new Vector3[_boundaryPointCount];
        var expectedNormals = new Vector3[_boundaryPointCount];
        var expectedUvs = new Vector2[_boundaryPointCount];
        var expectedIndexCount = (_boundaryPointCount - 2) * 3;

        for (var i = 0; i < _boundaryPointCount; i++)
        {
            expectedNormals[i] = new Vector3(0, 0, 1);
            expectedVertices[i] = new Vector3(-boundary[i].x, boundary[i].y, 0);
            expectedUvs[i] = new Vector2(-boundary[i].x, boundary[i].y);
        }

        var mesh = _meshFilter.sharedMesh;
        Assert.AreEqual(expectedNormals, mesh.normals);
        Assert.AreEqual(expectedVertices, mesh.vertices);
        Assert.AreEqual(expectedUvs, mesh.uv);
        Assert.AreEqual(expectedIndexCount, mesh.GetIndices(0).Length);
        Assert.LessOrEqual(mesh.bounds.extents.x, 1);
        Assert.LessOrEqual(mesh.bounds.extents.y, 1);
        Assert.Zero(mesh.bounds.extents.z);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator MeshTriangleNormalsArePositiveZ()
    {
        _boundaryPointCount = 10;
        _planeMesh.enabled = _plane.enabled = true;
        yield return WaitForMeshAndVerifyResults();
        var mesh = _meshFilter.sharedMesh;

        var triangles = mesh.triangles;
        var vertices = mesh.vertices;

        for (var i = 0; i < triangles.Length; i += 3)
        {
            var a = vertices[triangles[i + 0]];
            var b = vertices[triangles[i + 1]];
            var c = vertices[triangles[i + 2]];

            var edgeAB = b - a;
            var edgeBC = c - b;
            var normal = Vector3.Cross(edgeAB, edgeBC);

            // For scene anchors, triangle normals are expected to point along positive +Z
            Assert.Positive(Vector3.Dot(Vector3.forward, normal));
        }
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator BoundaryVerticesAreInClockwiseOrder()
    {
        _boundaryPointCount = 3;
        _plane.enabled = true;
        yield return new WaitUntil(() => _plane.Boundary.Count == 3);
        var vertices = _plane.Boundary;

        var edge01 = vertices[1] - vertices[0];
        var edge12 = vertices[2] - vertices[1];
        var normal = Vector3.Cross(edge01, edge12);
        Assert.Positive(Vector3.Dot(Vector3.back, normal));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator MeshGenerationProducesCorrectMesh()
    {
        _planeMesh.enabled = true;
        _plane.enabled = true;
        yield return null;

        for (var i = 3; i < 8; i++)
        {
            yield return GenerateMesh(i);
        }

        // Couple stress tests
        yield return GenerateMesh(50);
        yield return GenerateMesh(100);
        yield return GenerateMesh(200);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator EnableDisableCanBeSafelyCycled()
    {
        _boundaryPointCount = 10;

        using (new LockScope(1))
        {
            _planeMesh.enabled = true;
            _plane.enabled = true;
            yield return null;

            Assert.IsTrue(_plane._jobHandle.HasValue);
            Assert.IsFalse(_plane._jobHandle.Value.IsCompleted);

            // Disabling should queue up dispose jobs and set the job handle back to null
            _plane.enabled = false;
            Assert.IsFalse(_plane._jobHandle.HasValue);
        }

        // When we re-enable the component, it should wait for the job to complete and then clear the data.
        _plane.enabled = true;
        yield return new WaitWhile(() => _plane._jobHandle.HasValue);

        Assert.Zero(_plane.Boundary.Count);

        // Now show we can generate a new mesh
        yield return GenerateMesh(100);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator GetBoundaryFailsGracefully()
    {
        _boundaryPointCount = 10;
        _plane.enabled = true;
        _failGetBoundaryOnSecondCall = true;
        _plane.RequestBoundary();
        yield return null;
        yield return new WaitUntil(() => _plane._jobHandle == null);
        Assert.Zero(_plane.Boundary.Count);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanSafelyDestroyImmediateWhileRunning()
    {
        _boundaryPointCount = 10;
        JobHandle jobHandle;
        using (new LockScope(1))
        {
            _plane.enabled = true;
            yield return new WaitUntil(() => _plane._jobHandle.HasValue);
            Assert.IsTrue(_plane._jobHandle.HasValue);
            Assert.IsFalse(_plane._jobHandle.Value.IsCompleted);
            jobHandle = _plane._jobHandle.Value;
        }

        Object.DestroyImmediate(_plane);
        yield return new WaitUntil(() => jobHandle.IsCompleted);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanSafelyDisableAndDestroy()
    {
        _planeMesh.enabled = true;
        yield return null;

        _boundaryPointCount = 10;

        JobHandle jobHandle;
        using (new LockScope(1))
        {
            _plane.enabled = true;
            yield return new WaitUntil(() => _plane._jobHandle.HasValue);
            Assert.IsTrue(_plane._jobHandle.HasValue);
            Assert.IsFalse(_plane._jobHandle.Value.IsCompleted);
            jobHandle = _plane._jobHandle.Value;
            _plane.enabled = false;
            Object.Destroy(_plane);
        }

        yield return new WaitUntil(() => jobHandle.IsCompleted);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator StartGeneratesAMesh()
    {
        Assert.IsFalse(_planeMesh.JobHandleForTesting.HasValue);
        _boundaryPointCount = 10;
        _planeMesh.enabled = true;
        _plane.enabled = true;
        yield return WaitForMeshAndVerifyResults();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanRequestMeshWhileGenerating()
    {
        _planeMesh.enabled = true;
        _plane.enabled = true;
        yield return null;

        _boundaryPointCount = 10;
        _plane.RequestBoundary();
        yield return new WaitWhile(() => _plane.Boundary.Count == 0);
        Assert.True(_planeMesh.MeshRequested);
        yield return WaitForMeshAndVerifyResults();
        Assert.True(_planeMesh.MeshRequested);
        yield return WaitForMeshAndVerifyResults();
    }
}

#endif
