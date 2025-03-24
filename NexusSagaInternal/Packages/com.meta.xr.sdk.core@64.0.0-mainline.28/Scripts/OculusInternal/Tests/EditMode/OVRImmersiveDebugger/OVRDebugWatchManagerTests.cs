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
using System;
using Meta.XR.ImmersiveDebugger.Manager;
using System.Collections;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

// The class has to be UnityEngine components that's active in the scene to be discovered
internal class TestClassWithWatchAttribute : MonoBehaviour
{
    [DebugMember]
    public int numberData = 0;

    [DebugMember]
    public static bool boolData;

    [DebugMember]
    public float FloatData { get; set; }

    [DebugMember]
    public Vector3 Vector3Data { get; set; }
}

internal class OVRDebugWatchManagerTests : OVRDebugManagerTestBase<TestClassWithWatchAttribute, WatchManager>
{
    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        TestClassWithWatchAttribute.boolData = false;
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
        Assert.IsTrue(Manager.WatchesDict.ContainsKey(typeof(TestClassWithWatchAttribute)));
        Assert.AreEqual(Manager.WatchesDict[typeof(TestClassWithWatchAttribute)].Count, 4);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestRegisteredTypes()
    {
        foreach (var (type, watchType) in WatchUtils.Types)
        {
            TestRegisteredType(type);
            yield return null;
        }
    }

    public void TestRegisteredType(Type type)
    {
        var method = this.GetType().GetMethod("TestRegisteredTypeGeneric");
        var generic = method.MakeGenericMethod(type);
        generic.Invoke(this, null);
    }

    public void TestRegisteredTypeGeneric<T>()
    {
        Watch<T>.ResetBuffer();
        var numberOfDisplayStrings = Watch<T>.NumberOfDisplayStrings;
        Assert.That(numberOfDisplayStrings, Is.GreaterThan(0));
        var defaultStrings = Watch<T>.ToDisplayStrings(default);
        for (var i = 0; i < numberOfDisplayStrings; i++)
        {
            Assert.That(defaultStrings[i], Is.Not.Null);
        }
        Assert.That(numberOfDisplayStrings, Is.EqualTo(defaultStrings.Length));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdate()
    {
        yield return ProcessType();

        var index = 0;
        foreach (var instance in Instances)
        {
            instance.numberData = index;
            instance.FloatData = index / 10.0f;
            instance.Vector3Data = new Vector3(index, index + 1, index + 2);

            index++;
        }

        TestClassWithWatchAttribute.boolData = true;

        index = 0;
        foreach (var instance in Instances)
        {
            TestWatch(instance, "numberData", Watch<int>.ToDisplayStrings(index));
            TestWatch(instance, "FloatData", Watch<float>.ToDisplayStrings(index / 10.0f));
            TestWatch(instance, "Vector3Data", Watch<Vector3>.ToDisplayStrings(new Vector3(index, index + 1, index + 2)));

            index++;
        }
        TestWatch(null, "boolData", Watch<bool>.ToDisplayStrings(true));
        yield return null;
    }

    private void TestWatch(TestClassWithWatchAttribute instance, string memberName, string[] expectedData)
    {
        var member = GetMember(instance, memberName);
        var watch = member.GetWatch();
        var value = watch.Value;
        var values = watch.Values;
        Assert.IsNotNull(value);
        Assert.That(value, Is.EqualTo(values[0]));
        for (var i = 0; i < watch.NumberOfValues; i++)
        {
            Assert.That(values[i], Is.EqualTo(expectedData[i]));
        }
    }
}

#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
