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
public class OVRCustomSkeletonTests
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
    }

    [Test]
    public void TestAutomappingFromOVRBody()
    {
        var characterGo = LoadPrefab("OVRCustomBodyPrefabTest");
        var customSkeleton = characterGo.GetComponent<OVRCustomSkeleton>();

        Assert.NotNull(customSkeleton);
        Assert.AreEqual(OVRSkeleton.SkeletonType.Body, customSkeleton.GetSkeletonType());

        var expectedBoneMapping = GetBoneMappingDictionary(customSkeleton);

        ClearBoneMapping(customSkeleton);

        // re-map it
        customSkeleton.AutoMapBonesFromOculusSkeleton();

        ValidateBoneMapping(customSkeleton, expectedBoneMapping);
    }

    [Test]
    public void TestAutomappingWithOculusSkeleton()
    {
        var characterGo = LoadPrefab("OVRCustomBodyPrefabTest");
        var customSkeleton = characterGo.GetComponent<OVRCustomSkeleton>();
        Assert.NotNull(customSkeleton);
        Assert.AreEqual(OVRSkeleton.SkeletonType.Body, customSkeleton.GetSkeletonType());
    }

    [Test]
    public void TestAutomappingWithUnityHumanoid()
    {
        var characterGo = LoadPrefab("ArmatureSkinningUpdateRetarget");
        var retargeter = characterGo.GetComponent<OVRUnityHumanoidSkeletonRetargeter>();
        Assert.NotNull(retargeter);
        Assert.AreEqual(OVRSkeleton.SkeletonType.Body, retargeter.GetSkeletonType());
    }

    private GameObject LoadPrefab(string prefabPath)
    {
        var prefab = Resources.Load(prefabPath) as GameObject;
        Assert.NotNull(prefab);

        var characterGo = Object.Instantiate(prefab);
        Assert.NotNull(characterGo);

        _loadedObjects.Add(characterGo);

        return characterGo;
    }

    private static void ValidateBoneMapping(OVRCustomSkeleton customSkeleton,
        Dictionary<OVRSkeleton.BoneId, Transform> expectedBoneMapping)
    {
        var start = customSkeleton.GetCurrentStartBoneId();
        var end = customSkeleton.GetCurrentEndBoneId();
        if (customSkeleton.IsValidBone(start) && customSkeleton.IsValidBone(end))
        {
            for (int i = (int)start; i < (int)end; ++i)
            {
                Assert.AreEqual(true, expectedBoneMapping.TryGetValue((OVRSkeleton.BoneId)i, out var expectedBone));
                Assert.AreEqual(expectedBone, customSkeleton.CustomBones[i]);
            }
        }
    }

    private static void ClearBoneMapping(OVRCustomSkeleton customSkeleton)
    {
        for (var i = 0; i < customSkeleton.CustomBones.Count; i++)
        {
            customSkeleton.CustomBones[i] = null;
        }
    }

    private static Dictionary<OVRSkeleton.BoneId, Transform> GetBoneMappingDictionary(OVRCustomSkeleton customSkeleton)
    {
        var ret = new Dictionary<OVRSkeleton.BoneId, Transform>();

        var start = customSkeleton.GetCurrentStartBoneId();
        var end = customSkeleton.GetCurrentEndBoneId();

        if (!customSkeleton.IsValidBone(start) || !customSkeleton.IsValidBone(end))
        {
            return ret;
        }

        for (var i = (int)start; i < (int)end; ++i)
        {
            ret[(OVRSkeleton.BoneId)i] = customSkeleton.CustomBones[i];
        }

        return ret;
    }
}

#endif // OVRPLUGIN_TESTING
