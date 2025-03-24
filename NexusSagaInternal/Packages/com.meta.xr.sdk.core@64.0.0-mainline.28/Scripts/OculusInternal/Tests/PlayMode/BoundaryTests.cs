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

#if OVR_INTERNAL_CODE
#if OVRPLUGIN_TESTING

using NUnit.Framework;
using System.Collections;
using UnityEngine.TestTools;
using static OVRPlugin;

[TestFixture]
internal class BoundaryTests : OVRPluginPlayModeTest
{
    [UnityTest]
    public IEnumerator BoundaryVisibilityTest()
    {
        Assert.AreEqual(OVRPlugin.RequestBoundaryVisibility(BoundaryVisibility.Suppressed), Result.Success);
        Assert.That(FakeOVRPlugin_93.Visible, Is.False);

        Assert.AreEqual(OVRPlugin.RequestBoundaryVisibility(BoundaryVisibility.NotSuppressed), Result.Success);
        Assert.That(FakeOVRPlugin_93.Visible, Is.True);

        yield return null;
    }

    [UnityTest]
    public IEnumerator BoundaryMustBeVisibleTest()
    {
        FakeOVRPlugin_93.PTEnabled = false;

        var result = OVRPlugin.RequestBoundaryVisibility(BoundaryVisibility.Suppressed);
        Assert.AreEqual(result, Result.Warning_BoundaryVisibilitySuppressionNotAllowed);
        Assert.That(FakeOVRPlugin_93.Visible, Is.True);

        FakeOVRPlugin_93.PTEnabled = true;

        yield return null;
    }

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        OVRPlugin.OVRP_1_93_0.mockObj = new FakeOVRPlugin_93();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        yield return base.UnityTearDown();
        OVRPlugin.OVRP_1_93_0.mockObj = new OVRPlugin.OVRP_1_93_0_TEST();
    }

    class FakeOVRPlugin_93 : OVRP_1_93_0_TEST
    {
        public static bool PTEnabled = true;
        public static bool Visible = true;

        public override Result ovrp_RequestBoundaryVisibility(BoundaryVisibility boundaryVisibility)
        {
            if (!PTEnabled && boundaryVisibility == BoundaryVisibility.Suppressed) {
                Visible = true;
                return Result.Warning_BoundaryVisibilitySuppressionNotAllowed;
            }

            Visible = boundaryVisibility == BoundaryVisibility.NotSuppressed;
            return Result.Success;
        }
    }
}

#endif
#endif
