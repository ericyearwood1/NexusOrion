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
using System.Collections;
using UnityEngine.TestTools;

internal class OVRManagerPassthroughPreferencesTest : OVRPluginEditModeTest
{
    private FakeOVRPlugin87 _fakeOVRPlugin87;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        _fakeOVRPlugin87 = new FakeOVRPlugin87();
        OVRPlugin.OVRP_1_87_0.mockObj = _fakeOVRPlugin87;
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        OVRPlugin.OVRP_1_87_0.mockObj = new OVRPlugin.OVRP_1_87_0_TEST();
        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestDefaultToActiveFlag()
    {
        _fakeOVRPlugin87.PreferencesDelegate = GetDefaultToActiveTrue;
        Assert.IsTrue(OVRManager.IsPassthroughRecommended());

        _fakeOVRPlugin87.PreferencesDelegate = GetDefaultToActiveFalse;
        Assert.IsFalse(OVRManager.IsPassthroughRecommended());

        yield return null;
    }

    private OVRPlugin.Result GetDefaultToActiveTrue(out OVRPlugin.PassthroughPreferences preferences)
    {
        preferences = default;
        preferences.Flags = OVRPlugin.PassthroughPreferenceFlags.DefaultToActive;
        return OVRPlugin.Result.Success;
    }

    private OVRPlugin.Result GetDefaultToActiveFalse(out OVRPlugin.PassthroughPreferences preferences)
    {
        preferences = default;
        preferences.Flags = 0;
        return OVRPlugin.Result.Success;
    }

    private delegate OVRPlugin.Result GetPassthroughPreferencesHandler(out OVRPlugin.PassthroughPreferences preferences);
    private class FakeOVRPlugin87 : OVRPlugin.OVRP_1_87_0_TEST
    {
        public GetPassthroughPreferencesHandler PreferencesDelegate { get; set; }

        public override OVRPlugin.Result ovrp_GetPassthroughPreferences(out OVRPlugin.PassthroughPreferences preferences)
        {
            if(PreferencesDelegate != null)
            {
                return PreferencesDelegate(out preferences);
            }
            preferences = default;
            return OVRPlugin.Result.Failure;
        }
    }
}

#endif // OVRPLUGIN_TESTING
