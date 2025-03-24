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
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using Object = UnityEngine.Object;

[TestFixture]
public class OVRCustomFaceTests
{
    private List<GameObject> _loadedObjects;

    [SetUp]
    public void SetUp()
    {
        _loadedObjects = new List<GameObject>();
    }

    [TearDown]
    public void CleanUp()
    {
        foreach (var go in _loadedObjects)
        {
            Object.DestroyImmediate(go);
        }

        _loadedObjects = null;
    }

    [Test]
    public void TestAutomappingFromOculusFace()
    {
        TestFaceModelForRetargeting("OVRCustomBodyPrefabTest", OVRCustomFace.RetargetingType.OculusFace);
    }

    private void TestFaceModelForRetargeting(string prefabName, OVRCustomFace.RetargetingType retargeting)
    {
        var faceGo = LoadPrefab(prefabName);
        var customFace = faceGo.GetComponentInChildren<OVRCustomFace>();

        Assert.NotNull(customFace);
        customFace.retargetingType = retargeting;

        var expectedExpressionMapping = GetExpressionMapping(customFace);
        customFace.ClearBlendshapes();

        // re-map it
        customFace.AutoMapBlendshapes();
        var actualExpressionMapping = GetExpressionMapping(customFace);

        Assert.AreEqual(expectedExpressionMapping, actualExpressionMapping);
    }

    private GameObject LoadPrefab(string prefabPath)
    {
        var prefab = Resources.Load(prefabPath) as GameObject;
        Assert.NotNull(prefab);

        var gameObject = Object.Instantiate(prefab);
        Assert.NotNull(gameObject);

        _loadedObjects.Add(gameObject);

        return gameObject;
    }

    private static List<OVRFaceExpressions.FaceExpression> GetExpressionMapping(OVRCustomFace customFace)
    {
        var mapping = new List<OVRFaceExpressions.FaceExpression>();
        var renderer = customFace.GetComponent<SkinnedMeshRenderer>();
        Assert.NotNull(renderer);

        for (int i = 0; i < renderer.sharedMesh.blendShapeCount; ++i)
        {
            mapping.Add(customFace._mappings[i]);
        }

        return mapping;
    }
}

#endif // OVRPLUGIN_TESTING
