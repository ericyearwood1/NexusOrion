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
internal class TestClass : MonoBehaviour
{
    public Pose poseData = new Pose();
    public static Pose staticPoseData = new Pose();
}


internal class OVRDebugGizmoRendererManagerTests : OVRPluginPlayModeTest
{
    private GameObject _gizmoObject;
    private InstanceCache _instanceCache;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        GizmoTypesRegistry.InitGizmos();
        _gizmoObject = new GameObject("Gizmo Object");
        _instanceCache = new InstanceCache();
        _instanceCache.RegisterClassType(typeof(TestClass));
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        GizmoTypesRegistry.Reset();
        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestStart()
    {
        var rendererManager = _gizmoObject.AddComponent<GizmoRendererManager>();
        var classType = typeof(TestClass);
        MemberInfo member = classType.GetField("poseData");
        rendererManager.Setup(classType, member, DebugGizmoType.Axis, Color.black, _instanceCache);
        yield return null;

        var gizmoRenderers = _gizmoObject.GetComponents<GizmoRenderer>();
        Assert.AreEqual(gizmoRenderers.Length, 1);
        Assert.IsFalse(gizmoRenderers[0].enabled);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestSetState()
    {
        var testClass = _gizmoObject.AddComponent<TestClass>();
        _instanceCache.RetrieveInstances();

        var rendererManager = _gizmoObject.AddComponent<GizmoRendererManager>();
        var classType = typeof(TestClass);
        MemberInfo member = classType.GetField("poseData");
        rendererManager.Setup(classType, member, DebugGizmoType.Axis, Color.black, _instanceCache);
        yield return null;

        var gizmoRenderers = _gizmoObject.GetComponents<GizmoRenderer>();
        rendererManager.SetState(testClass, true);
        yield return null;
        Assert.IsTrue(gizmoRenderers[0].enabled);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateForTwoInstances()
    {
        _gizmoObject.AddComponent<TestClass>();
        _gizmoObject.AddComponent<TestClass>();
        _instanceCache.RetrieveInstances();

        var rendererManager = _gizmoObject.AddComponent<GizmoRendererManager>();
        var classType = typeof(TestClass);
        MemberInfo member = classType.GetField("poseData");
        rendererManager.Setup(classType, member, DebugGizmoType.Axis, Color.black, _instanceCache);
        yield return null;
        var gizmoRenderers = _gizmoObject.GetComponents<GizmoRenderer>();
        Assert.AreEqual(gizmoRenderers.Length, 2);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateForTwoStatics()
    {
        _gizmoObject.AddComponent<TestClass>();
        _gizmoObject.AddComponent<TestClass>();
        _instanceCache.RetrieveInstances();

        var rendererManager = _gizmoObject.AddComponent<GizmoRendererManager>();
        var classType = typeof(TestClass);
        MemberInfo member = classType.GetField("staticPoseData");
        rendererManager.Setup(classType, member, DebugGizmoType.Axis, Color.black, _instanceCache);
        yield return null;
        var gizmoRenderers = _gizmoObject.GetComponents<GizmoRenderer>();
        Assert.AreEqual(gizmoRenderers.Length, 1);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateForDynamicInstances()
    {
        var rendererManager = _gizmoObject.AddComponent<GizmoRendererManager>();
        var classType = typeof(TestClass);
        MemberInfo member = classType.GetField("poseData");
        rendererManager.Setup(classType, member, DebugGizmoType.Axis, Color.black, _instanceCache);
        yield return null;
        Assert.AreEqual(_gizmoObject.GetComponents<GizmoRenderer>().Length, 1);

        // dynamic increasing
        var comp1 = _gizmoObject.AddComponent<TestClass>();
        var comp2 = _gizmoObject.AddComponent<TestClass>();
        var comp3 = _gizmoObject.AddComponent<TestClass>();
        var comp4 = _gizmoObject.AddComponent<TestClass>();
        _instanceCache.RetrieveInstances();
        rendererManager.SetState(comp1, true);
        rendererManager.SetState(comp2, true);
        rendererManager.SetState(comp3, true);
        rendererManager.SetState(comp4, true);
        yield return null;
        Assert.AreEqual(_gizmoObject.GetComponents<GizmoRenderer>().Length, 4);

        // dynamic decreasing
        UnityEngine.Object.DestroyImmediate(comp1);
        UnityEngine.Object.DestroyImmediate(comp2);
        _instanceCache.RetrieveInstances();
        yield return null;
        var renderers = _gizmoObject.GetComponents<GizmoRenderer>();
        Assert.AreEqual(renderers.Length, 4);
        Assert.IsTrue(renderers[0].enabled);
        Assert.IsTrue(renderers[1].enabled);
        Assert.IsFalse(renderers[2].enabled);
        Assert.IsFalse(renderers[3].enabled);
    }
}

#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
