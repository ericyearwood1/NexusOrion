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
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

public class CustomFaceTests : OVRPluginPlayModeTest
{
    private FakeOVRPlugin_78 _fakeOVRPlugin78;
    private FakeOVRPlugin_92 _fakeOVRPlugin92;
    private FakeOVRPluginConstants _fakeConstants;

    private OVRFaceExpressions _faceExpressions;

    private static string[] _mockBlendshapeNames = { "foo", "bar", "baz" };
    private List<GameObject> _loadedObjects;

#region UNITY TESTS

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        _fakeConstants = new FakeOVRPluginConstants();
        foreach (OVRFaceExpressions.FaceExpression expression in Enum.GetValues(
                     typeof(OVRFaceExpressions.FaceExpression)))
        {
            if (expression == OVRFaceExpressions.FaceExpression.Max ||
                expression == OVRFaceExpressions.FaceExpression.Invalid)
            {
                continue;
            }

            int id = (int)expression;
            float val = 0.01f * id;
            _fakeConstants._fakeSpecificWeights.Add($"{id}", val);
        }

        _fakeOVRPlugin78 = new FakeOVRPlugin_78(_fakeConstants);
        OVRPlugin.OVRP_1_78_0.mockObj = _fakeOVRPlugin78;
        _fakeOVRPlugin92 = new FakeOVRPlugin_92(_fakeConstants);
        OVRPlugin.OVRP_1_92_0.mockObj = _fakeOVRPlugin92;

        GameObject FaceTrackingGO = new GameObject();
        _faceExpressions = FaceTrackingGO.AddComponent<OVRFaceExpressions>();

        var _meshRenderer = FaceTrackingGO.AddComponent<SkinnedMeshRenderer>();
        _meshRenderer.sharedMesh = OVRMockMeshCreator.GenerateMockMesh(_mockBlendshapeNames);

        _loadedObjects = new List<GameObject>();

        //Wait at least one update is called
        yield return null;
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        GameObject.Destroy(_faceExpressions.gameObject);
        yield return null;

        OVRPlugin.OVRP_1_78_0.mockObj = new OVRPlugin.OVRP_1_78_0_TEST();
        OVRPlugin.OVRP_1_82_0.mockObj = new OVRPlugin.OVRP_1_82_0_TEST();

        foreach (var go in _loadedObjects)
        {
            GameObject.Destroy(go);
        }

        _loadedObjects = null;

        yield return base.UnityTearDown();
    }

    private GameObject LoadPrefab(string prefabPath)
    {
        var prefab = Resources.Load(prefabPath) as GameObject;
        Assert.NotNull(prefab);

        var gameObject = GameObject.Instantiate(prefab);
        Assert.NotNull(gameObject);

        _loadedObjects.Add(gameObject);

        return gameObject;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestGetAllFaceExpressionValues()
    {
        foreach (OVRFaceExpressions.FaceExpression expression
                 in Enum.GetValues(typeof(OVRFaceExpressions.FaceExpression)))
        {
            if (expression == OVRFaceExpressions.FaceExpression.Max ||
                expression == OVRFaceExpressions.FaceExpression.Invalid)
            {
                continue;
            }

            int id = (int)expression;
            bool result = _faceExpressions.TryGetFaceExpressionWeight(expression, out var weight);
            Assert.IsTrue(result);
            Assert.AreEqual(0.01f * id, weight);
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestAddedMesh()
    {
        var gameObject = _faceExpressions.gameObject;
        Assert.IsNotNull(gameObject);
        var meshRenderer = gameObject.GetComponent<SkinnedMeshRenderer>();
        Assert.IsNotNull(meshRenderer);
        Assert.AreEqual(_mockBlendshapeNames.Length, meshRenderer.sharedMesh.blendShapeCount);

        yield return null;
    }


    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator AddOVRCustomFace()
    {
        var gameObject = _faceExpressions.gameObject;
        var meshRenderer = gameObject.GetComponent<SkinnedMeshRenderer>();

        var customFace = gameObject.AddComponent<OVRCustomFace>();
        OVRFaceExpressions.FaceExpression[] mapping =
        {
            OVRFaceExpressions.FaceExpression.CheekSuckL /* 6 */,
            OVRFaceExpressions.FaceExpression.EyesLookDownL /* 14 */,
            OVRFaceExpressions.FaceExpression.LipCornerPullerL /* 32 */
        };
        customFace.Mappings = mapping;
        customFace.BlendShapeStrengthMultiplier = 50;

        // wait for the next frame
        yield return null;

        // a just reminder that FaceExpression values are 0.01f * id, see UnitySetUp() above
        // and multiplier is 50
        Assert.AreEqual(3f, meshRenderer.GetBlendShapeWeight(0));
        Assert.AreEqual(7f, meshRenderer.GetBlendShapeWeight(1));
        Assert.AreEqual(16f, meshRenderer.GetBlendShapeWeight(2));

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator AddOVRMappedCustomFace()
    {
        // We need to load a prefab with actual blendshapes on it for
        // the mapping test to work.
        var gameObject = LoadPrefab("OVRCustomBodyPrefabTest");

        var customFace = gameObject.GetComponentInChildren<OVRCustomFace>();
        customFace._allowDuplicateMapping = false;
        customFace.ClearBlendshapes();
        customFace.AutoMapBlendshapes();

        // make sure at least one blendshape got mapped
        Assert.IsNotNull(customFace.Mappings);
        Assert.IsTrue(customFace.Mappings.Length > 0);
        bool foundAtLeastOneMappedValue = false;
        foreach (var mapping in customFace.Mappings)
        {
            if (mapping != OVRFaceExpressions.FaceExpression.Invalid &&
                mapping != OVRFaceExpressions.FaceExpression.Max)
            {
                foundAtLeastOneMappedValue = true;
            }
        }
        Assert.IsTrue(foundAtLeastOneMappedValue);

        yield return null;
    }

#endregion
}

#endif
