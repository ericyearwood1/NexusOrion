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

using System.Collections;
using NUnit.Framework;
using UnityEditor;
using UnityEngine.TestTools;

[InitializeOnLoad]
internal class InitializeOnLoadPlaymodeTests : OVRPluginPlayModeTest
{
    private static bool _runtimeAssetsWasLoadedEarly = false;
    private static bool _onEditorReadyCalled = false;

    static InitializeOnLoadPlaymodeTests()
    {
        Meta.XR.Editor.Callbacks.InitializeOnLoad.OnEditorReadyEarly +=
            () => _runtimeAssetsWasLoadedEarly = OVRRuntimeAssetsBase.LoadedAssetInstance;
        Meta.XR.Editor.Callbacks.InitializeOnLoad.Register(
            () => _onEditorReadyCalled = true);
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestOnEditorReadyCalled()
    {
        Assert.IsTrue(_onEditorReadyCalled);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestRuntimeAssetsLoaded()
    {
        // This unit test would fail if any child of OVRRuntimeAssetsBase (e.g. OVRRuntimeSettings) is accessed early than assets loaded.
        // Use InitializeOnLoad.Register from Meta.XR.Editor.Callbacks to resolve.
        Assert.IsFalse(_runtimeAssetsWasLoadedEarly);
        yield return null;
    }
}

#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
