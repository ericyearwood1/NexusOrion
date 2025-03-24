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
using Meta.XR.ImmersiveDebugger.UserInterface;
using Meta.XR.ImmersiveDebugger.Utils;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

// The class has to be UnityEngine components that's active in the scene to be discovered
internal class TestClassWithTweakAttribute : MonoBehaviour
{
    [DebugMember(Tweakable = true, Min = 1.0f, Max = 2.0f)]
    public float floatData;
    [DebugMember(Tweakable = true, Min = 3.0f, Max = 4.0f)]
    public static float staticFloatData;
    [DebugMember(Tweakable = true)]
    public bool boolData;
    [DebugMember(Tweakable = true)]
    public static bool staticBoolData;

    [DebugMember(Tweakable = false)]
    private bool _boolData;
    [DebugMember(Tweakable = true)]
    public bool BoolData
    {
        get
        {
            return _boolData;
        }
        set
        {
            _boolData = value;
        }
    }

    public void SetupInitialValues()
    {
        staticFloatData = 3.0f;
        staticBoolData = true;
    }
}

internal class OVRDebugTweakManagerTests : OVRDebugManagerTestBase<TestClassWithTweakAttribute, TweakManager>
{
    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        foreach (var instance in Instances)
        {
            instance.SetupInitialValues();
        }
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
        Assert.IsTrue(Manager.TweaksDict.ContainsKey(typeof(TestClassWithTweakAttribute)));
        Assert.AreEqual(Manager.TweaksDict[typeof(TestClassWithTweakAttribute)].Count, 5);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestChangeValue()
    {
        yield return ProcessType();

        var index = 0;
        foreach (var instance in Instances)
        {
            instance.floatData = index / 1.0f;
            instance.boolData = (index % 2) == 0;
            instance.BoolData = (index % 2) == 0;

            index++;
        }

        TestClassWithTweakAttribute.staticBoolData = true;
        TestClassWithTweakAttribute.staticFloatData = 3.5f;

        index = 0;
        foreach (var instance in Instances)
        {
            TestTweak<float>(instance, "floatData", index / 1.0f);
            TestTweak<bool>(instance, "boolData", (index % 2) == 0);
            TestTweak<bool>(instance, "BoolData", (index % 2) == 0);

            index++;
        }

        TestTweak<float>(null, "staticFloatData", 3.5f);
        TestTweak<bool>(null, "staticBoolData", true);

        yield return null;
    }

    private void TestTweak<T>(TestClassWithTweakAttribute instance, string memberName, T expectedValue)
    {
        var memberInfo = typeof(TestClassWithTweakAttribute).GetMember(memberName,
            BindingFlags.Public | BindingFlags.Static | BindingFlags.Instance)[0];
        var attribute = memberInfo.GetCustomAttribute<DebugMember>();
        var member = GetMember(instance, memberName);
        var tweak = member.GetTweak();
        Assert.IsNotNull(tweak);
        var min = Tweak<T>.FromFloat(attribute.Min);
        var max = Tweak<T>.FromFloat(attribute.Max);

        ReadTweak(tweak, min, max, expectedValue);
        ChangeTweak(instance, tweak, min, max, memberInfo, 0.0f);
        ChangeTweak(instance, tweak, min, max, memberInfo, 0.5f);
        ChangeTweak(instance, tweak, min, max, memberInfo, 1.0f);
    }

    private void ReadTweak<T>(Tweak tweak, T min, T max, T expectedValue)
    {
        var tween = tweak.Tween;
        var expectedTween = Tweak<T>.InverseLerp(min, max, expectedValue);
        Assert.That(tween, Is.EqualTo(expectedTween));
    }

    private void ChangeTweak<T>(TestClassWithTweakAttribute instance, Tweak tweak, T min, T max, MemberInfo memberInfo, float newTween)
    {
        tweak.Tween = newTween;
        var expectedNewValue = Tweak<T>.Lerp(min, max, newTween);
        Assert.That((T)memberInfo.GetValue(instance), Is.EqualTo(expectedNewValue));
    }
}
#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
