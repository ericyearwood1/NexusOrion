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
using Meta.XR.ImmersiveDebugger;
using Meta.XR.ImmersiveDebugger.DebugData;
using Meta.XR.ImmersiveDebugger.Editor;
using NUnit.Framework;
using System.Collections;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using Assert = UnityEngine.Assertions.Assert;

internal class TestClassForCompilationProcessor : MonoBehaviour
{
    [DebugMember]
    public bool someBoolean;
}

internal class OVRDebugCompilationProcessorTests : OVRPluginEditModeTest
{
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestCompilationCallback()
    {
        var cachedName = RuntimeSettings.InstanceAssetName;
        RuntimeSettings.InstanceAssetName = "TestCompilationProcessor";

        var currentAssembly = typeof(OVRDebugCompilationProcessorTests).Assembly;
        CompilationProcessor.OnAssemblyCompilationEnded(currentAssembly.Location);

        var debugTypesDict = RuntimeSettings.Instance.debugTypesDict;

        Assert.IsTrue(debugTypesDict.ContainsKey(currentAssembly.GetName().Name));
        Assert.IsTrue(debugTypesDict[currentAssembly.GetName().Name].Contains("TestClassForCompilationProcessor"));

        AssetDatabase.DeleteAsset(RuntimeSettings.GetDebugTypesInstanceAssetPath());
        RuntimeSettings.InstanceAssetName = cachedName;
        yield return null;
    }
}

#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
