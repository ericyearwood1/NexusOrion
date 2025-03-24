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
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

// The class has to be UnityEngine components that's active in the scene to be discovered
internal class TestClassWithActionAttribute : MonoBehaviour
{
    public bool instanceMethodCalled = false;
    public static bool staticMethodCalled = false;
    [DebugMember]
    public void TestInstanceMethod()
    {
        instanceMethodCalled = true;
    }
    [DebugMember]
    public static void TestStaticMethod()
    {
        staticMethodCalled = true;
    }
}

internal class OVRDebugActionManagerTests : OVRDebugManagerTestBase<TestClassWithActionAttribute, ActionManager>
{
    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        TestClassWithActionAttribute.staticMethodCalled = false;
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestProcessType()
    {
        yield return ProcessType();
        Assert.AreEqual(Manager.ActionsDict.Count, 1);
        Assert.AreEqual(Manager.ActionsDict[typeof(TestClassWithActionAttribute)].Count, 2);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestButtonClick()
    {
        yield return ProcessType();

        foreach (var instance in Instances)
        {
            TestMethod(instance, true);
        }
        TestMethod(null, true);
    }

    private void TestMethod(TestClassWithActionAttribute instance, bool click)
    {
        var member = GetMember(instance, instance ? "TestInstanceMethod" : "TestStaticMethod");
        var action = member.GetAction();
        Assert.IsNotNull(action);

        if (!click) return;

        action.Invoke();
        Assert.IsTrue(instance == null
            ? TestClassWithActionAttribute.staticMethodCalled
            : instance.instanceMethodCalled);
    }
}
#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
