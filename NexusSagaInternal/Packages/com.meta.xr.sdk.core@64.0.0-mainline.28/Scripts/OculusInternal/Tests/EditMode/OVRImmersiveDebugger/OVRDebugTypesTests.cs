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
#if OVR_INTERNAL_CODE
using Meta.XR.ImmersiveDebugger.DebugData;
using NUnit.Framework;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using Assert = UnityEngine.Assertions.Assert;

internal class OVRDebugTypesTests : OVRPluginEditModeTest
{
    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        RuntimeSettings.ClearInstance();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestSerializationDeserialization()
    {
        var testAssetPath = "Assets/TestDebugTypes.asset";
        RuntimeSettings originalRuntimeSettings = ScriptableObject.CreateInstance<RuntimeSettings>();
        originalRuntimeSettings.debugTypesDict = new Dictionary<string, List<string>>();
        originalRuntimeSettings.debugTypesDict["tmpKey"] = new List<string>()
        {
            "tmpType1",
            "tmpType2"
        };

        AssetDatabase.CreateAsset(originalRuntimeSettings, testAssetPath);
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        RuntimeSettings loadedRuntimeSettings = AssetDatabase.LoadAssetAtPath<RuntimeSettings>(testAssetPath);

        Assert.IsNotNull(loadedRuntimeSettings);
        Assert.IsNotNull(loadedRuntimeSettings.debugTypesDict);
        Assert.AreEqual(originalRuntimeSettings.debugTypesDict, loadedRuntimeSettings.debugTypesDict);

        AssetDatabase.DeleteAsset(testAssetPath);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestAssetInstanceCreation()
    {
        var testAssetName = "TestDebugTypes";
        var cachedExistingInstanceAssetName = RuntimeSettings.InstanceAssetName;
        RuntimeSettings.InstanceAssetName = testAssetName;
        var expectedAssetPath = "Assets/Resources/TestDebugTypes.asset";
        Assert.AreEqual(RuntimeSettings.GetDebugTypesInstanceAssetPath(), expectedAssetPath);
        var assetExistedFunc = new Func<bool>(() => !string.IsNullOrEmpty(AssetDatabase.AssetPathToGUID(expectedAssetPath, AssetPathToGUIDOptions.OnlyExistingAssets)));

        Assert.IsFalse(assetExistedFunc());

        // getting instance, create the asset under the hood
        Assert.IsNotNull(RuntimeSettings.Instance);

        Assert.IsTrue(assetExistedFunc());

        // recover
        AssetDatabase.DeleteAsset(expectedAssetPath);
        RuntimeSettings.InstanceAssetName = cachedExistingInstanceAssetName;
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateDebugTypesDict()
    {
        RuntimeSettings testRuntimeSettings = ScriptableObject.CreateInstance<RuntimeSettings>();
        testRuntimeSettings.debugTypesDict = new Dictionary<string, List<string>>();
        testRuntimeSettings.debugTypesDict["tmpKey1"] = new List<string>()
        {
            "tmpType1",
            "tmpType2"
        };

        // new tmpKey2 assembly found
        RuntimeSettings.UpdateTypes("tmpKey2", new List<string>
        {
            "tmpType3",
            "tmpType4"
        }, testRuntimeSettings);

        Assert.IsTrue(testRuntimeSettings.debugTypesDict.ContainsKey("tmpKey2"));
        Assert.IsTrue(testRuntimeSettings.debugTypesDict["tmpKey2"].Contains("tmpType3"));
        Assert.IsTrue(testRuntimeSettings.debugTypesDict["tmpKey2"].Contains("tmpType4"));

        // update existing tmpKey1
        RuntimeSettings.UpdateTypes("tmpKey1", new List<string>
        {
            "tmpType5"
        }, testRuntimeSettings);
        Assert.IsTrue(testRuntimeSettings.debugTypesDict.ContainsKey("tmpKey1"));
        Assert.IsFalse(testRuntimeSettings.debugTypesDict["tmpKey1"].Contains("tmpType1"));
        Assert.IsTrue(testRuntimeSettings.debugTypesDict["tmpKey1"].Contains("tmpType5"));

        // tmpKey1 assembly no longer has interested types
        RuntimeSettings.UpdateTypes("tmpKey1", new List<string> {}, testRuntimeSettings);
        Assert.IsFalse(testRuntimeSettings.debugTypesDict.ContainsKey("tmpKey1"));

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateAllDebugTypes()
    {
        RuntimeSettings testRuntimeSettings = ScriptableObject.CreateInstance<RuntimeSettings>();
        testRuntimeSettings.debugTypesDict["tmpKey1"] = new List<string>()
        {
            "tmpType1",
            "tmpType2"
        };

        var updatedDebugTypes = new List<Type>();
        updatedDebugTypes.Add(typeof(TestClassWithAttribute)); // EditModeTests
        updatedDebugTypes.Add(typeof(UIEditorSetup)); // Oculus.VR

        RuntimeSettings.UpdateAllDebugTypes(updatedDebugTypes, testRuntimeSettings);

        Assert.IsFalse(testRuntimeSettings.debugTypesDict.ContainsKey("tmpKey1")); // old one is cleaned
        Assert.IsTrue(testRuntimeSettings.debugTypesDict.ContainsKey(typeof(TestClassWithAttribute).Assembly.GetName().Name));
        Assert.IsTrue(testRuntimeSettings.debugTypesDict.ContainsKey(typeof(UIEditorSetup).Assembly.GetName().Name));
        Assert.IsTrue(testRuntimeSettings.debugTypesDict[typeof(TestClassWithAttribute).Assembly.GetName().Name].Contains("TestClassWithAttribute"));
        Assert.IsTrue(testRuntimeSettings.debugTypesDict[typeof(UIEditorSetup).Assembly.GetName().Name].Contains("UIEditorSetup"));
        yield return null;
    }
}

#endif //OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
