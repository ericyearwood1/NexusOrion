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
using Meta.XR.ImmersiveDebugger.Manager;
using Meta.XR.ImmersiveDebugger.UserInterface;
using Meta.XR.ImmersiveDebugger.Utils;
using System.Collections;
using NUnit.Framework;
using NUnit.Framework.Internal;
using System;
using System.Reflection;
using UnityEngine;
using UnityEngine.TestTools;
using Assert = UnityEngine.Assertions.Assert;

// The class has to be UnityEngine components that's active in the scene to be discovered
internal class TestClassWithGizmoAttribute : MonoBehaviour
{
    [DebugMember(GizmoType = DebugGizmoType.Axis)]
    public Pose poseData = new Pose();

    [DebugMember(GizmoType = DebugGizmoType.Point)]
    public static Vector3 pointData = new Vector3();

    private Tuple<Vector3, Vector3> _lineData = new Tuple<Vector3, Vector3>(new Vector3(), new Vector3());
    [DebugMember(GizmoType = DebugGizmoType.Line)]
    public Tuple<Vector3, Vector3> LineData
    {
        get
        {
            return _lineData;
        }
        set
        {
            _lineData = value;
        }
    }
}

internal class OVRDebugGizmoManagerTests : OVRDebugManagerTestBase<TestClassWithGizmoAttribute, GizmoManager>
{
    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        GizmoTypesRegistry.RegisterGizmoType(DebugGizmoType.Axis, typeof(Pose), dataSource => {});
        GizmoTypesRegistry.RegisterGizmoType(DebugGizmoType.Point, typeof(Vector3), dataSource => {});
        GizmoTypesRegistry.RegisterGizmoType(DebugGizmoType.Line, typeof(Tuple<Vector3, Vector3>), dataSource => {});
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        GizmoTypesRegistry.Reset();
        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestProcessType()
    {
        yield return ProcessType();
        Assert.IsTrue(Manager.GizmosDict.ContainsKey(typeof(TestClassWithGizmoAttribute)));
        Assert.AreEqual(Manager.GizmosDict[typeof(TestClassWithGizmoAttribute)].Count, 3);
    }
}
#endif //OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
