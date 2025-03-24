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
using Meta.XR.ImmersiveDebugger.Manager;
using System.Collections;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

// The class has to be UnityEngine components that's active in the scene to be discovered
internal class TestClassWithAttribute : MonoBehaviour
{
    [DebugMember(DebugColor.Red)]
    public void TestInstanceMethod() {}
}

internal class OVRDebugAttributeTests : OVRDebugManagerTestBase<TestClassWithAttribute, ActionManager>
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
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestAttributeValues()
    {
        yield return ProcessType();

        foreach (var instance in Instances)
        {
            TestAttribute(instance);
        }
    }

    private void TestAttribute(TestClassWithAttribute instance)
    {
        var member = GetMember(instance, "TestInstanceMethod");
        var attribute = member.GetAttribute();
        Assert.IsNotNull(attribute);

        var memberInfo =
            typeof(TestClassWithAttribute).GetMethod("TestInstanceMethod", BindingFlags.Instance | BindingFlags.Public);
        var memberInfoAttribute = memberInfo?.GetCustomAttribute<DebugMember>();
        Assert.IsNotNull(memberInfoAttribute);
        Assert.AreEqual(attribute.Color, memberInfoAttribute.Color);
        Assert.AreEqual(attribute.Max, memberInfoAttribute.Max);
        Assert.AreEqual(attribute.Min, memberInfoAttribute.Min);
        Assert.AreEqual(attribute.Tweakable, memberInfoAttribute.Tweakable);
        Assert.AreEqual(attribute.GizmoType, memberInfoAttribute.GizmoType);
        Assert.AreEqual(Color.red, attribute.Color);
    }
}
#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
