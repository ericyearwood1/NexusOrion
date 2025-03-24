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

using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using UnityEngine.TestTools.Utils;

[Category("OnCall:xrinput_text_entry")]
internal class OVRGLTFLoaderTests
{
    public struct OVRGLTFModelTestCase
    {
        public string glbFilename;
        public bool supportAnimation;
        public int nodeCount;
        public int animationNodeCount;
        public int animationNodeIndexCount;
        public int morphTargetCount;
        public int textureUriHandlerInvokeCount;
        public int firstMeshVertexCount;
        public int totalMeshVertexCount;
        public int totalActiveMeshVertexCount;

        public int firstMeshColorsLength;
        public int firstMeshWeightsLength;
        public Vector4 firstMeshWeight;
        public int bufferIndexReadTextureBufferLength;
        public int firstAccessorVector3ArrayLength;
        public int firstAccessorVector2ArrayLength;
        public int firstAccessorVector4ArrayLength;
        public int firstAccessorMatrixArrayLength;
        public int firstAccessorIntArrayLength;
        public int firstAccessorFloatArrayLength;
        public Vector3 firstAccessorVector3ArrayItem0;
        public Vector3 firstAccessorVector3ArrayItem1;
        public Vector2 firstAccessorVector2ArrayItem;
        public Vector4 firstAccessorVector4ArrayItem;
        public Matrix4x4 firstAccessorMatrixArrayItem;
        public int firstAccessorIntArrayItem;
        public float firstAccessorFloatArrayItem;
        public Vector3 firstMeshBoundsSize;
    }

    public const string ModelRoot = "Assets/Oculus/VR/Scripts/OculusInternal/Tests/PlayMode/Resources/";

    private static readonly OVRGLTFModelTestCase ControllerOvrgltfModelTestCase = new()
    {
        glbFilename = "rtouch_controller.glb",
        supportAnimation = true,
        nodeCount = 13,
        animationNodeCount = 6,
        animationNodeIndexCount = 1,
        firstMeshVertexCount = 3305,
        totalMeshVertexCount = 3305,
        totalActiveMeshVertexCount = 3305,
        firstMeshBoundsSize = new Vector3(0.0678520724f, 0.0834731013f, 0.109254509f),

        firstMeshWeightsLength = 4,
        firstMeshWeight = new Vector4(1,0,0,0 ),
        bufferIndexReadTextureBufferLength = 8356,
        firstAccessorVector3ArrayLength = 4,
        firstAccessorVector2ArrayLength = 4,
        firstAccessorVector4ArrayLength = 4,
        firstAccessorMatrixArrayLength = 7,
        firstAccessorIntArrayLength = 6,
        firstAccessorFloatArrayLength = 97,

        firstAccessorVector3ArrayItem0 = new Vector3(-0.0355452262f, -0.00155899406f, 0.0043803174f),
        firstAccessorVector3ArrayItem1 = new Vector3(-0.0335892886f, -0.000973425573f, 0.00438031694f),
        firstAccessorVector2ArrayItem = new Vector2(0,1),
        firstAccessorVector4ArrayItem = new Vector4(0,0, 0, 0),
        firstAccessorMatrixArrayItem = new Matrix4x4(
            new Vector4(-1,0,0,0 ),
            new Vector4(0,-1,0,0 ),
            new Vector4(0,0,1,0 ),
            new Vector4(0,0,0,1 )),
        firstAccessorIntArrayItem = 0,
        firstAccessorFloatArrayItem = 0,
    };

    private static readonly OVRGLTFModelTestCase VirtualKeyboardOvrgltfModelTestCase = new()
    {
        glbFilename = "virtual_keyboard.glb",
        supportAnimation = false, // skip for performance reasons
        nodeCount = 1488,
        morphTargetCount = 368,
        textureUriHandlerInvokeCount = 3,
        firstMeshVertexCount = 144,
        totalMeshVertexCount = 160632,
        totalActiveMeshVertexCount = 21760,
        firstMeshBoundsSize = new Vector3(1f, 0.445512831f, 0f),

        bufferIndexReadTextureBufferLength = 32,
        firstAccessorVector3ArrayLength = 2,
        firstAccessorVector2ArrayLength = 4,
        firstAccessorVector4ArrayLength = 144,
        firstAccessorMatrixArrayLength = 0,
        firstAccessorIntArrayLength = 168,
        firstAccessorFloatArrayLength = 2,

        firstAccessorVector3ArrayItem0 = new Vector3(0,0,0),
        firstAccessorVector3ArrayItem1 = new Vector3(1, 1, 1),
        firstAccessorVector2ArrayItem = new Vector2(1,0),
        firstAccessorVector4ArrayItem = new Vector4(1,0, 0, 1),
        firstAccessorIntArrayItem = 0,
        firstAccessorFloatArrayItem = 0,
    };

    private static readonly OVRGLTFModelTestCase TrackedKeyboardOvrgltfModelTestCase = new()
    {
        glbFilename = "apple_macbook_13_Lid_AlphaBlend_sm_glb.bytes",
        nodeCount = 2,
        firstMeshVertexCount = 4267,
        totalMeshVertexCount = 4675,
        totalActiveMeshVertexCount = 4675,
        firstMeshBoundsSize = new Vector3(0.301350892f, 0.186492473f, 0.0877617598f),

        firstMeshColorsLength = 4267,
        bufferIndexReadTextureBufferLength = 422041,
        firstAccessorVector3ArrayLength = 4267,
        firstAccessorVector2ArrayLength = 4267,
        firstAccessorVector4ArrayLength = 4267,
        firstAccessorMatrixArrayLength = 0,
        firstAccessorIntArrayLength = 11460,
        firstAccessorFloatArrayLength = 0,

        firstAccessorVector3ArrayItem0 = new Vector3(0.140981868f, 0.0340806395f, -0.0442185327f),
        firstAccessorVector3ArrayItem1 = new Vector3(-0.0794485882f, -0.0609103516f, -0.00175178051f),
        firstAccessorVector2ArrayItem = new Vector2(0.462873578f, 0.875528753f),
        firstAccessorVector4ArrayItem = new Vector4(65535, 65535, 65535, 65535),
        firstAccessorIntArrayItem = 1
    };

    private static readonly OVRGLTFModelTestCase TrackedKeyboardK830OvrgltfModelTestCase = new()
    {
        glbFilename = "k830_glb.bytes",
        nodeCount = 2,
        firstMeshVertexCount = 4807,
        totalMeshVertexCount = 4807,
        totalActiveMeshVertexCount = 4807,
        firstMeshBoundsSize = new Vector3(0.364441007f, 0.0160000008f, 0.124774992f),

        firstMeshColorsLength = 4807,
        bufferIndexReadTextureBufferLength = 377893,
        firstAccessorVector3ArrayLength = 4807,
        firstAccessorVector2ArrayLength = 4807,
        firstAccessorVector4ArrayLength = 4807,
        firstAccessorMatrixArrayLength = 0,
        firstAccessorIntArrayLength = 8982,
        firstAccessorFloatArrayLength = 0,

        firstAccessorVector3ArrayItem0 = new Vector3(-0.177728623f, -0.00700000115f, -0.0620170012f),
        firstAccessorVector3ArrayItem1 = new Vector3(-0.174413025f, -0.00400000252f, -0.0620170012f),
        firstAccessorVector2ArrayItem = new Vector2(0.0431698486f, 0.628480315f),
        firstAccessorVector4ArrayItem = new Vector4(65535f, 65535f, 65535f, 65535f),
        firstAccessorIntArrayItem = 741
    };

    public static readonly OVRGLTFModelTestCase[] TestCaseData = new[]
    {
        ControllerOvrgltfModelTestCase,
        VirtualKeyboardOvrgltfModelTestCase,
        TrackedKeyboardOvrgltfModelTestCase,
        TrackedKeyboardK830OvrgltfModelTestCase
    };


    [MenuItem("Oculus/Internal/OVRGLTFLoader/Load Test Assets Into Scene")]
    public static void EditorLoadAllTestAssets()
    {
        Debug.Log("Loading glTF test assets into scene for visual inspection");
        foreach (var testCase in TestCaseData) {
            var loader = new OVRGLTFLoader(ModelRoot + testCase.glbFilename);
            var result = loader.LoadGLB(true, true);
            result.root.hideFlags = HideFlags.DontSave;
        }
    }

    private int _textureUriHandlerInvokesReceived;
    private OVRGLTFLoader _loader;
    private static IEnumerable GLTFModelTestCases
    {
        get
        {
            foreach (var testCase in TestCaseData)
            {
                yield return new TestCaseData(testCase).SetName(testCase.glbFilename);
            }
        }
    }

    [TearDown]
    public void TearDown()
    {
        if (_loader != null && _loader.scene.root != null)
        {
            GameObject.DestroyImmediate(_loader.scene.root);
        }
        _loader = null;
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void LoadGLBLoadsModel(OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        InitializeLoader(ovrgltfModelTestCase);

        _loader.LoadGLB(ovrgltfModelTestCase.supportAnimation);

        AssertOVRGLTFScene(ovrgltfModelTestCase);
    }

    [Test, TestCaseSource(nameof(GLTFModelTestCases))]
    public void LoadGLBLoadsModelAsync(OVRGLTFModelTestCase ovrgltfModelTestCase)
    {
        InitializeLoader(ovrgltfModelTestCase);

        var loadGlbCoroutine = _loader.LoadGLBCoroutine(ovrgltfModelTestCase.supportAnimation);
        while (loadGlbCoroutine.MoveNext())
        {
            // unwrap coroutine synchronously
        }
        AssertOVRGLTFScene(ovrgltfModelTestCase);
    }

    private void InitializeLoader(OVRGLTFModelTestCase modelTestCase)
    {
        _textureUriHandlerInvokesReceived = 0;
        var tempTexture = new Texture2D(1, 1);
        _loader =
            new OVRGLTFLoader(ModelRoot + modelTestCase.glbFilename);
        _loader.textureUriHandler += (s, mat) =>
        {
            Assert.That(s, Does.StartWith("metaVirtualKeyboard://texture/"));
            _textureUriHandlerInvokesReceived++;
            return tempTexture;
        };
    }

    private void AssertOVRGLTFScene(OVRGLTFModelTestCase modelTestCase)
    {
        var scene = _loader.scene;
        Assert.That(scene.root, Is.Not.Null);
        Assert.That(scene.root.activeInHierarchy, Is.True);
        Assert.That(scene.animationNodes.Count, Is.EqualTo(modelTestCase.animationNodeCount));
        Assert.That(scene.animationNodeLookup.Count, Is.EqualTo(modelTestCase.animationNodeIndexCount));
        Assert.That(scene.morphTargetHandlers.Count, Is.EqualTo(modelTestCase.morphTargetCount));

        Assert.That(_textureUriHandlerInvokesReceived, Is.EqualTo(modelTestCase.textureUriHandlerInvokeCount));
#if OVR_INTERNAL_CODE
        Assert.That(scene.jsonParsingDurationMs, Is.GreaterThan(0));

        var skinnedMeshRenderers = scene.root.GetComponentsInChildren<SkinnedMeshRenderer>(true);
        var meshFilters = scene.root.GetComponentsInChildren<MeshFilter>(true);
        var firstMesh = (skinnedMeshRenderers.Length > 0) ? skinnedMeshRenderers[0].sharedMesh : meshFilters[0].sharedMesh;
        Assert.That(firstMesh.vertexCount, Is.EqualTo(modelTestCase.firstMeshVertexCount));
        Assert.That(firstMesh.bounds.size, Is.EqualTo(modelTestCase.firstMeshBoundsSize).Using(Vector3EqualityComparer.Instance));

        var totalMeshVertexCount = skinnedMeshRenderers.Sum(mesh => mesh.sharedMesh.vertexCount);
        totalMeshVertexCount += meshFilters.Sum(mesh => mesh.sharedMesh.vertexCount);
        Assert.That(totalMeshVertexCount, Is.EqualTo(modelTestCase.totalMeshVertexCount));

        var activeSkinnedMeshRenderers = scene.root.GetComponentsInChildren<SkinnedMeshRenderer>(false);
        var activeMeshFilters = scene.root.GetComponentsInChildren<MeshFilter>(false);
        var totalActiveMeshVertexCount = activeSkinnedMeshRenderers.Sum(mesh => mesh.sharedMesh.vertexCount);
        totalActiveMeshVertexCount += activeMeshFilters.Sum(mesh => mesh.sharedMesh.vertexCount);
        Assert.That(totalActiveMeshVertexCount, Is.EqualTo(modelTestCase.totalActiveMeshVertexCount));
#endif
    }

}

#endif
