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

using Meta.XR.ImmersiveDebugger.Gizmo;
using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Assert = UnityEngine.Assertions.Assert;

internal class OVRDebugGizmosTests : OVRPluginEditModeTest
{
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestDrawWithColorScope()
    {
        // ensure DrawWithColor restore previous color, and correctly change the Color during drawing func invocation.
        DebugGizmos.Color = Color.white; // default color
        var drawingColor = Color.black;
        using (var colorScope = new DebugGizmos.ColorScope(Color.red))
        {
            drawingColor = DebugGizmos.Color;
        }
        Assert.AreEqual(DebugGizmos.Color, Color.white);
        Assert.AreEqual(drawingColor, Color.red);
        yield return null;
    }
}
#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
