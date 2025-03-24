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
using Meta.XR.ImmersiveDebugger.Gizmo;
using System.Collections;
using NUnit.Framework;
using System;
using UnityEngine;
using UnityEngine.TestTools;
using Assert = UnityEngine.Assertions.Assert;

internal class OVRDebugGizmoTypesTests : OVRPluginEditModeTest
{
    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        yield return base.UnityTearDown();
        GizmoTypesRegistry.Reset();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestGizmoDataSourceTypeChecking()
    {
        // checking a randomly assigned types for gizmo, irrelevant to their actual types
        GizmoTypesRegistry.RegisterGizmoType(DebugGizmoType.Axis, typeof(float), o => {});
        Assert.IsTrue(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(float), DebugGizmoType.Axis));
        Assert.IsFalse(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(Pose), DebugGizmoType.Axis));

        GizmoTypesRegistry.RegisterGizmoType(DebugGizmoType.Line, typeof(Vector3), o => {});
        Assert.IsTrue(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(Vector3), DebugGizmoType.Line));
        Assert.IsFalse(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(Pose), DebugGizmoType.Line));
        Assert.IsFalse(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(float), DebugGizmoType.Line));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestInitGizmosWithTypeChecking()
    {
        // checking pre-registered gizmo types
        GizmoTypesRegistry.InitGizmos();
        Assert.IsTrue(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(Pose), DebugGizmoType.Axis));
        Assert.IsFalse(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(float), DebugGizmoType.Axis));
        Assert.IsTrue(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(Vector3), DebugGizmoType.Point));
        Assert.IsTrue(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(Tuple<Vector3, Vector3>), DebugGizmoType.Line));
        Assert.IsTrue(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(Tuple<Pose, float, float>), DebugGizmoType.Plane));
        Assert.IsTrue(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(Tuple<Vector3, float>), DebugGizmoType.Cube));
        Assert.IsTrue(GizmoTypesRegistry.IsValidDataTypeForGizmoType(typeof(Tuple<Pose, float, float, float>), DebugGizmoType.Box));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestRenderDelegateBeingCalledAsExpected()
    {
        bool delegateCalled = false;
        int dataSource = 0;

        GizmoTypesRegistry.RegisterGizmoType(DebugGizmoType.Axis, typeof(float), d =>
        {
            delegateCalled = true;
            dataSource = (int)d;
        });
        GizmoTypesRegistry.RenderGizmo(DebugGizmoType.Axis, 1);
        Assert.IsTrue(delegateCalled);
        Assert.AreEqual(dataSource, 1);
        yield return null;
    }
}

#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
