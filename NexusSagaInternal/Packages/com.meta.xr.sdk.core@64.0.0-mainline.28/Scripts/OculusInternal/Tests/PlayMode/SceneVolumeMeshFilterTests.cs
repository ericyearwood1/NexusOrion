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
using System.Collections;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.TestTools;
using static OVRPlugin;

internal unsafe class SceneVolumeMeshFilterTests : OVRPluginPlayModeTest
{
    // fixture level variables
    private NativeArray<Vector3> _vertices;
    private NativeArray<int> _indices;
    private GameObject _camera;

    // test level variables
    private GameObject _gameObject;

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CreateVolumeMesh()
    {
        var volumeMeshFilter = CreateVolumeMeshObject(false);

        yield return new WaitWhile(() => !volumeMeshFilter.IsCompleted);

        var meshFilter = volumeMeshFilter.GetComponent<MeshFilter>();
        var mesh = meshFilter.sharedMesh;

        // test counts, winding order & openxr conversion
        Assert.IsTrue(volumeMeshFilter.IsCompleted);
        Assert.AreEqual(mesh.vertexCount, 8);
        Assert.AreEqual(mesh.GetIndexCount(0), 36);
        Assert.AreEqual(mesh.vertices[0], new Vector3(0, 0, 0));
        Assert.AreEqual(mesh.vertices[2], new Vector3(-1, 1, 0));
        Assert.AreEqual(mesh.vertices[6], new Vector3(-1, 1, 1));
        Assert.AreEqual(mesh.GetIndices(0)[1], 1);
        Assert.AreEqual(mesh.GetIndices(0)[2], 2);
        Assert.AreEqual(mesh.GetIndices(0)[31], 6);
        Assert.AreEqual(mesh.GetIndices(0)[32], 7);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CreateVolumeMeshWithCollider()
    {
        var volumeMeshFilter = CreateVolumeMeshObject(true);

        yield return new WaitWhile(() => !volumeMeshFilter.IsCompleted);

        var meshFilter = volumeMeshFilter.GetComponent<MeshFilter>();
        var meshCollider = volumeMeshFilter.GetComponent<MeshCollider>();

        Assert.AreEqual(meshFilter.sharedMesh.vertexCount, 8);
        Assert.AreEqual(meshCollider.sharedMesh, meshFilter.sharedMesh);

        // test collision properties
        var eyePosition = new Vector3(0, 0, -5);
        var directionToMesh = (meshCollider.bounds.center - eyePosition).normalized;
        Assert.IsTrue(Physics.Raycast(eyePosition, directionToMesh, out var hitInfo));
        Assert.AreEqual(hitInfo.collider, meshCollider);
        Assert.IsTrue(meshCollider.bounds.size == Vector3.one); // == for approx.

        // ensure normal is pointing the right way
        Assert.Positive(Vector3.Dot(-hitInfo.normal, directionToMesh));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator AccessVolumeMeshWhileGenerating()
    {
        var volumeMeshFilter = CreateVolumeMeshObject(true);
        var meshFilter = volumeMeshFilter.GetComponent<MeshFilter>();
        var meshCollider = volumeMeshFilter.GetComponent<MeshCollider>();

        // let Start() run on our test game objects
        yield return null;

        Assert.IsNotNull(meshFilter.sharedMesh);
        Assert.AreEqual(meshFilter.sharedMesh.vertexCount, 0);
        Assert.IsNull(meshCollider.sharedMesh);
        Assert.IsFalse(volumeMeshFilter.IsCompleted);

        yield return new WaitWhile(() => !volumeMeshFilter.IsCompleted);

        Assert.AreEqual(meshFilter.sharedMesh.vertexCount, 8);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator VolumeMeshFailNicely()
    {
        using var noVertices = new NativeArray<Vector3>(0, Allocator.Persistent);
        using var noIndices = new NativeArray<int>(0, Allocator.Persistent);
        OVRP_1_82_0.mockObj = new Mock82 { Vertices = noVertices, Indices = noIndices };

        var volumeMeshFilter = CreateVolumeMeshObject(false);
        var meshFilter = volumeMeshFilter.GetComponent<MeshFilter>();

        yield return new WaitWhile(() => !volumeMeshFilter.IsCompleted);

        Assert.AreEqual(meshFilter.sharedMesh.vertexCount, 0);

        OVRP_1_82_0.mockObj = new Mock82 { Vertices = _vertices, Indices = _indices };
    }

    private OVRSceneVolumeMeshFilter CreateVolumeMeshObject(bool withCollider)
    {
        _gameObject = new GameObject("Volume Mesh",
            typeof(OVRSceneAnchor),
            typeof(OVRSceneVolume),
            typeof(MeshFilter),
            typeof(OVRSceneVolumeMeshFilter));
        if (withCollider)
            _gameObject.AddComponent<MeshCollider>();
        _gameObject.GetComponent<OVRSceneAnchor>().Initialize(new OVRAnchor(new OVRSpace(1), Guid.NewGuid()));

        return _gameObject.GetComponent<OVRSceneVolumeMeshFilter>();
    }

    [TearDown]
    public void TearDown()
    {
        if (_gameObject != null) UnityEngine.Object.DestroyImmediate(_gameObject);
    }

    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        _vertices = new NativeArray<Vector3>(8, Allocator.Persistent);
        _indices = new NativeArray<int>(36, Allocator.Persistent);
        _vertices.CopyFrom(CubeVertices());
        _indices.CopyFrom(CubeIndices());

        // we need the camera for OVRSceneAnchor transforms
        _camera = new GameObject("Camera", typeof(Camera))
        {
            tag = "MainCamera"
        };

        OVRP_1_64_0.mockObj = new Mock64();
        OVRP_1_82_0.mockObj = new Mock82 { Vertices = _vertices, Indices = _indices };
    }

    [OneTimeTearDown]
    public void OneTimeTearDown()
    {
        OVRP_1_64_0.mockObj = new OVRP_1_64_0_TEST();
        OVRP_1_82_0.mockObj = new OVRP_1_82_0_TEST();
        _vertices.Dispose();
        _indices.Dispose();

        if (_camera != null) UnityEngine.Object.DestroyImmediate(_camera);
    }

#region Mock implementations

    private class Mock64 : OVRP_1_64_0_TEST
    {
        public override Result ovrp_LocateSpace(ref Posef location, ref UInt64 space, TrackingOrigin trackingOrigin)
        {
            location = Posef.identity;
            return Result.Success;
        }
    }

    private class Mock82 : OVRP_1_82_0_TEST
    {
        public NativeArray<Vector3> Vertices;
        public NativeArray<int> Indices;

        public override Result ovrp_GetSpaceTriangleMesh(ref ulong space, ref TriangleMeshInternal mesh)
        {
            mesh.vertexCapacityInput = Vertices.Length;
            mesh.vertexCountOutput = Vertices.Length;
            mesh.indexCapacityInput = Indices.Length;
            mesh.indexCountOutput = Indices.Length;

            // if it's not 0x0, then we have to populate the pre-assigned arrays
            if (mesh.vertices != IntPtr.Zero)
            {
                var vertexSize = Marshal.SizeOf(typeof(Vector3)) * Vertices.Length;
                var indexSize = Marshal.SizeOf(typeof(int)) * Indices.Length;
                Buffer.MemoryCopy(Vertices.GetUnsafePtr(), mesh.vertices.ToPointer(), vertexSize, vertexSize);
                Buffer.MemoryCopy(Indices.GetUnsafePtr(), mesh.indices.ToPointer(), indexSize, indexSize);
            }

            return Result.Success;
        }
    }

#endregion

#region Mock data

    private Vector3[] CubeVertices()
    {
        return new Vector3[]
        {
            new Vector3(0, 0, 0),
            new Vector3(0, 1, 0),
            new Vector3(1, 1, 0),
            new Vector3(1, 0, 0),
            new Vector3(0, 0, 1),
            new Vector3(0, 1, 1),
            new Vector3(1, 1, 1),
            new Vector3(1, 0, 1),
        };
    }

    private int[] CubeIndices()
    {
        var faces = new int[][]
        {
            new int[] { 0, 2, 1, 0, 3, 2 }, // front
            new int[] { 4, 5, 6, 4, 6, 7 }, // back
            new int[] { 0, 4, 7, 0, 7, 3 }, // bottom
            new int[] { 1, 6, 5, 1, 2, 6 }, // top
            new int[] { 0, 5, 4, 0, 1, 5 }, // left
            new int[] { 3, 7, 6, 3, 6, 2 }, // right
        };
        return faces.SelectMany(item => item).ToArray();
    }

#endregion
}

#endif
