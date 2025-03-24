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
using UnityEngine.TestTools;
using UnityEngine;
using static OVRPlugin;
using Random = UnityEngine.Random;

class OVRTriangleMeshAnchorTests : OVRPluginPlayModeTest
{
    Mock _mock;

    OVRTriangleMesh _triangleMesh;

    Vector3[] _vertices;

    int[] _indices;

    class Mock : OVRP_1_82_0_TEST
    {
        public struct MeshData
        {
            public Vector3[] Vertices;
            public int[] Indices;
        }

        public readonly Dictionary<ulong, MeshData> MeshDatas = new Dictionary<ulong, MeshData>();

        public override unsafe Result ovrp_GetSpaceTriangleMesh(ref UInt64 space,
            ref TriangleMeshInternal triangleMeshInternal)
        {
            if (!MeshDatas.TryGetValue(space, out var meshData)) return Result.Failure_InvalidParameter;

            var indices = meshData.Indices;
            var vertices = meshData.Vertices;

            if (triangleMeshInternal is { indexCapacityInput: 0, vertexCapacityInput: 0 })
            {
                triangleMeshInternal.indexCountOutput = indices.Length;
                triangleMeshInternal.vertexCountOutput = vertices.Length;
                return Result.Success;
            }

            if (triangleMeshInternal.indexCapacityInput == indices.Length &&
                triangleMeshInternal.vertexCapacityInput == vertices.Length)
            {
                if (triangleMeshInternal.vertices == IntPtr.Zero ||
                    triangleMeshInternal.indices == IntPtr.Zero)
                {
                    return Result.Failure_InvalidParameter;
                }

                fixed (void* vptr = vertices)
                {
                    UnsafeUtility.MemCpy((void*)triangleMeshInternal.vertices, vptr, sizeof(Vector3) * vertices.Length);
                }

                fixed (void* iptr = indices)
                {
                    UnsafeUtility.MemCpy((void*)triangleMeshInternal.indices, iptr, sizeof(int) * indices.Length);
                }

                return Result.Success;
            }

            return Result.Failure_InsufficientSize;
        }
    }

    static T FromSpace<T>(ulong space) where T : struct, IOVRAnchorComponent<T>
        => default(T).FromAnchor(new OVRAnchor(space, Guid.Empty));

    IEnumerable<int> TransformedIndices
    {
        get
        {
            for (var i = 0; i < _indices.Length; i += 3)
            {
                yield return _indices[i + 0];
                yield return _indices[i + 2];
                yield return _indices[i + 1];
            }
        }
    }

    IEnumerable<Vector3> TransformedVertices => _vertices.Select(v => new Vector3(-v.x, v.y, v.z));

    [Test]
    public void TryGetCountsProvidesExpectedCounts()
    {
        Assert.That(_triangleMesh.TryGetCounts(out var vertexCount, out var triangleCount), Is.True);
        Assert.That(vertexCount, Is.EqualTo(_vertices.Length));
        Assert.That(triangleCount * 3, Is.EqualTo(_indices.Length));
    }

    [Test]
    public void TryGetMeshRawUntransformedProvidesCorrectUntransformedMeshData()
    {
        using var vertexBuffer = new NativeArray<Vector3>(_vertices.Length, Allocator.TempJob);
        using var indexBuffer = new NativeArray<int>(_indices.Length, Allocator.TempJob);

        Assert.That(_triangleMesh.TryGetMeshRawUntransformed(vertexBuffer, indexBuffer), Is.True);
        Assert.That(vertexBuffer, Is.EqualTo(_vertices));
        Assert.That(indexBuffer, Is.EqualTo(_indices));
    }

    [Test]
    public void TryGetMeshProvidesCorrectTransformedMeshData()
    {
        using var vertexBuffer = new NativeArray<Vector3>(_vertices.Length, Allocator.TempJob);
        using var indexBuffer = new NativeArray<int>(_indices.Length, Allocator.TempJob);

        Assert.That(_triangleMesh.TryGetMesh(vertexBuffer, indexBuffer), Is.True);
        Assert.That(vertexBuffer, Is.EqualTo(TransformedVertices));
        Assert.That(indexBuffer, Is.EqualTo(TransformedIndices));
    }

    [Test]
    public void ScheduleGetMeshJobProducesCorrectTransformedMeshData()
    {
        using var vertexBuffer = new NativeArray<Vector3>(_vertices.Length, Allocator.TempJob);
        using var indexBuffer = new NativeArray<int>(_indices.Length, Allocator.TempJob);

        _triangleMesh.ScheduleGetMeshJob(vertexBuffer, indexBuffer).Complete();
        Assert.That(vertexBuffer, Is.EqualTo(TransformedVertices));
        Assert.That(indexBuffer, Is.EqualTo(TransformedIndices));
    }

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        _mock = new Mock();
        OVRP_1_82_0.mockObj = _mock;

        var space = (ulong)Random.Range(0, int.MaxValue) + (ulong)Random.Range(0, int.MaxValue);
        var n = 10;
        _vertices = new Vector3[n + 1];
        for (var i = 0; i < n; i++)
        {
            var a = (float)i / n * Mathf.PI * 2;
            _vertices[i] = new Vector3(Mathf.Cos(a), Mathf.Sin(a), 0);
        }

        _vertices[n] = Vector3.zero;

        _indices = new int[n * 3];
        for (var i = 0; i < n; i++)
        {
            _indices[i * 3 + 0] = n;
            _indices[i * 3 + 1] = i;
            _indices[i * 3 + 2] = (i + 1) % n;
        }

        _mock.MeshDatas[space] = new Mock.MeshData
        {
            Vertices = _vertices,
            Indices = _indices
        };

        _triangleMesh = FromSpace<OVRTriangleMesh>(space);
    }

    [OneTimeTearDown]
    public void OneTimeTearDown()
    {
        OVRP_1_82_0.mockObj = new OVRP_1_82_0_TEST();
        _mock = null;
    }
}

#endif
