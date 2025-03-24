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
using System.Drawing.Drawing2D;
using System.IO;
using System.Linq;
using NUnit.Framework;
using OVRSimpleJSON;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using UnityEngine.TestTools.Utils;
using File = UnityEngine.Windows.File;

[Category("OnCall:xrinput_text_entry")]
internal class OVRGLTFAccessorTests
{
    private Dictionary<string, JSONNode> _jsonDatas = new();
    private Dictionary<string, OVRGLTFAccessor> _accessor = new();

    private static IEnumerable GLTFModelTestCases
    {
        get
        {
            foreach (var testCase in OVRGLTFLoaderTests.TestCaseData)
            {
                yield return new TestCaseData(testCase).SetName(testCase.glbFilename);
            }
        }
    }
    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        foreach (var testCase in OVRGLTFLoaderTests.TestCaseData)
        {
            // Access JSON data within GLTF file
            var fileStream =
                new MemoryStream(File.ReadAllBytes(OVRGLTFLoaderTests.ModelRoot + testCase.glbFilename));
            OVRGLTFLoader.ValidateGLB(fileStream);
            byte[] jsonChunkData = OVRGLTFLoader.ReadChunk(fileStream, OVRChunkType.JSON);

            string json = System.Text.Encoding.ASCII.GetString(jsonChunkData);
            var jsonData = JSON.Parse(json);
            _jsonDatas[testCase.glbFilename] = jsonData;

            OVRGLTFAccessor.TryCreate(jsonData["accessors"], jsonData["bufferViews"], jsonData["buffers"],
            fileStream, out var accessor);
            _accessor[testCase.glbFilename] = accessor;
        }
    }

    private int AccessorWhere(JSONNode accessorsNode, Func<JSONNode, bool> condition)
    {
        var accessorIndex = 0;
        for (; accessorIndex < accessorsNode.Count; accessorIndex++)
        {
            if (condition.Invoke(accessorsNode[accessorIndex]))
            {
                return accessorIndex;
            }
        }
        return accessorIndex;
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void ReadAsTextureReturnsExpectedData(OVRGLTFLoaderTests.OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        var accessor = _accessor[ovrgltfModelTestCase.glbFilename];
        var bufferIndexReadTextureBufferLength = accessor.ReadBuffer(0).Length;
        Assert.That(bufferIndexReadTextureBufferLength, Is.EqualTo(ovrgltfModelTestCase.bufferIndexReadTextureBufferLength));
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void ReadAsVector3ReturnsExpectedData(OVRGLTFLoaderTests.OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        var jsonData = _jsonDatas[ovrgltfModelTestCase.glbFilename];
        var accessor = _accessor[ovrgltfModelTestCase.glbFilename];
        var accessorIndex = AccessorWhere(
            jsonData["accessors"],
            a => a["type"] == "VEC3"
            );
        accessor.Seek(accessorIndex);
        var vector3Array = accessor.ReadVector3(Vector3.one);

        Assert.That(vector3Array.Length, Is.EqualTo(ovrgltfModelTestCase.firstAccessorVector3ArrayLength));
        Assert.That(vector3Array[0], Is.EqualTo(ovrgltfModelTestCase.firstAccessorVector3ArrayItem0).Using(Vector3EqualityComparer.Instance));
        Assert.That(vector3Array[1], Is.EqualTo(ovrgltfModelTestCase.firstAccessorVector3ArrayItem1).Using(Vector3EqualityComparer.Instance));
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void ReadAsVector2ReturnsExpectedData(OVRGLTFLoaderTests.OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        var jsonData = _jsonDatas[ovrgltfModelTestCase.glbFilename];
        var accessor = _accessor[ovrgltfModelTestCase.glbFilename];
        var accessorIndex = AccessorWhere(
            jsonData["accessors"],
            a => a["type"] == "VEC2"
        );
        accessor.Seek(accessorIndex);
        var vector2Array = accessor.ReadVector2();

        Assert.That(vector2Array.Length, Is.EqualTo(ovrgltfModelTestCase.firstAccessorVector2ArrayLength));
        Assert.That(vector2Array[0], Is.EqualTo(ovrgltfModelTestCase.firstAccessorVector2ArrayItem).Using(Vector2EqualityComparer.Instance));
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void ReadAsVector4ReturnsExpectedData(OVRGLTFLoaderTests.OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        var jsonData = _jsonDatas[ovrgltfModelTestCase.glbFilename];
        var accessor = _accessor[ovrgltfModelTestCase.glbFilename];
        var accessorIndex = AccessorWhere(
            jsonData["accessors"],
            a => a["type"] == "VEC4"
        );
        accessor.Seek(accessorIndex);
        var vector4Array =  accessor.ReadVector4(Vector4.one);

        Assert.That(vector4Array.Length, Is.EqualTo(ovrgltfModelTestCase.firstAccessorVector4ArrayLength));
        Assert.That(vector4Array[0], Is.EqualTo(ovrgltfModelTestCase.firstAccessorVector4ArrayItem).Using(Vector4EqualityComparer.Instance));
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void ReadAsMatrix4X4ReturnsExpectedData(OVRGLTFLoaderTests.OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        var jsonData = _jsonDatas[ovrgltfModelTestCase.glbFilename];
        var accessor = _accessor[ovrgltfModelTestCase.glbFilename];
        var accessorIndex = AccessorWhere(
            jsonData["accessors"],
            a => a["type"] == "MAT4"
        );

        var matrix4Array = Array.Empty<Matrix4x4>();
        if (ovrgltfModelTestCase.firstAccessorMatrixArrayLength > 0)
        {
            accessor.Seek(accessorIndex);
            matrix4Array = accessor.ReadMatrix4x4(Vector3.one);
        }

        Assert.That(matrix4Array.Length, Is.EqualTo(ovrgltfModelTestCase.firstAccessorMatrixArrayLength));
        if (ovrgltfModelTestCase.firstAccessorMatrixArrayLength > 0)
        {
            Assert.That(matrix4Array[0].GetRow(0),
                Is.EqualTo(ovrgltfModelTestCase.firstAccessorMatrixArrayItem.GetRow(0))
                    .Using(Vector4EqualityComparer.Instance));
            Assert.That(matrix4Array[0].GetRow(1),
                Is.EqualTo(ovrgltfModelTestCase.firstAccessorMatrixArrayItem.GetRow(1))
                    .Using(Vector4EqualityComparer.Instance));
            Assert.That(matrix4Array[0].GetRow(2),
                Is.EqualTo(ovrgltfModelTestCase.firstAccessorMatrixArrayItem.GetRow(2))
                    .Using(Vector4EqualityComparer.Instance));
            Assert.That(matrix4Array[0].GetRow(3),
                Is.EqualTo(ovrgltfModelTestCase.firstAccessorMatrixArrayItem.GetRow(3))
                    .Using(Vector4EqualityComparer.Instance));
        }
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void ReadAsIntReturnsExpectedData(OVRGLTFLoaderTests.OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        var jsonData = _jsonDatas[ovrgltfModelTestCase.glbFilename];
        var accessor = _accessor[ovrgltfModelTestCase.glbFilename];
        var accessorIndex = AccessorWhere(
            jsonData["accessors"],
            a => a["type"] == "SCALAR"
                 && (OVRGLTFComponentType)a["componentType"].AsInt == OVRGLTFComponentType.UNSIGNED_SHORT
        );
        accessor.Seek(accessorIndex);
        var intArray = accessor.ReadInt();

        Assert.That(intArray.Length, Is.EqualTo(ovrgltfModelTestCase.firstAccessorIntArrayLength));
        Assert.That(intArray[0], Is.EqualTo(ovrgltfModelTestCase.firstAccessorIntArrayItem));
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void ReadAsFloatReturnsExpectedData(OVRGLTFLoaderTests.OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        var jsonData = _jsonDatas[ovrgltfModelTestCase.glbFilename];
        var accessor = _accessor[ovrgltfModelTestCase.glbFilename];
        var accessorIndex = AccessorWhere(
            jsonData["accessors"],
            a => a["type"] == "SCALAR"
                 && (OVRGLTFComponentType)a["componentType"].AsInt == OVRGLTFComponentType.FLOAT
        );

        var floatArray = Array.Empty<float>();
        if (ovrgltfModelTestCase.firstAccessorFloatArrayLength > 0)
        {
            accessor.Seek(accessorIndex);
            floatArray = accessor.ReadFloat();
        }

        Assert.That(floatArray.Length, Is.EqualTo(ovrgltfModelTestCase.firstAccessorFloatArrayLength));
        if (ovrgltfModelTestCase.firstAccessorFloatArrayLength > 0)
        {
            Assert.That(floatArray[0], Is.EqualTo(ovrgltfModelTestCase.firstAccessorFloatArrayItem));
        }
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void ReadAsColorAndReadAsBoneWeightsAsExpected(OVRGLTFLoaderTests.OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        var jsonData = _jsonDatas[ovrgltfModelTestCase.glbFilename];
        var accessor = _accessor[ovrgltfModelTestCase.glbFilename];

        // Create empty data to fill and later assert
        var firstMeshColors = Array.Empty<Color>();
        var firstMeshWeights = Array.Empty<BoneWeight>();

        // Search the model for the first mesh with colors or weights
        foreach (var mesh in jsonData["meshes"].Children)
        {
            foreach (var primitive in mesh["primitives"].Children)
            {
                if (firstMeshColors.Length == 0 && primitive["attributes"]["COLOR_0"])
                {
                    accessor.Seek(primitive["attributes"]["COLOR_0"].AsInt);
                    firstMeshColors = accessor.ReadColor();
                }
                if (firstMeshWeights.Length == 0 && primitive["attributes"]["WEIGHTS_0"])
                {
                    accessor.Seek(primitive["attributes"]["WEIGHTS_0"].AsInt);
                    firstMeshWeights = firstMeshWeights.Length == 0? new BoneWeight[accessor.GetDataCount()] : firstMeshWeights;
                    accessor.ReadWeights(ref firstMeshWeights);
                    accessor.Seek(primitive["attributes"]["JOINTS_0"].AsInt);
                    accessor.ReadJoints(ref firstMeshWeights);
                }
                if (firstMeshWeights.Length > 0 && firstMeshColors.Length > 0)
                    break;
            }
            if (firstMeshWeights.Length > 0 && firstMeshColors.Length > 0)
                break;
        }

        // Assert Results
        Assert.That(firstMeshColors.Length, Is.EqualTo(ovrgltfModelTestCase.firstMeshColorsLength));
        if (ovrgltfModelTestCase.firstMeshColorsLength > 0)
        {
            Assert.That(firstMeshColors[0].r, Is.EqualTo(1.0f));
            Assert.That(firstMeshColors[0].g, Is.EqualTo(1.0f));
            Assert.That(firstMeshColors[0].b, Is.EqualTo(1.0f));
            Assert.That(firstMeshColors[0].a, Is.EqualTo(1.0f));
        }
        Assert.That(firstMeshWeights.Length, Is.EqualTo(ovrgltfModelTestCase.firstMeshWeightsLength));
        if (ovrgltfModelTestCase.firstMeshWeightsLength > 0)
        {
            Assert.That(firstMeshWeights[0].weight0, Is.EqualTo(ovrgltfModelTestCase.firstMeshWeight.x));
            Assert.That(firstMeshWeights[0].weight1, Is.EqualTo(ovrgltfModelTestCase.firstMeshWeight.y));
            Assert.That(firstMeshWeights[0].weight2, Is.EqualTo(ovrgltfModelTestCase.firstMeshWeight.z));
            Assert.That(firstMeshWeights[0].weight3, Is.EqualTo(ovrgltfModelTestCase.firstMeshWeight.w));
        }
    }

}

#endif
