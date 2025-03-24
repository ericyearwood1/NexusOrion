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
using System;
using Meta.XR.ImmersiveDebugger.Utils;
using NUnit.Framework;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

internal class OVRDebugInstanceHandleTest : OVRPluginEditModeTest
{
    private static readonly Type Type = typeof(GameObject);
    private static GameObject Instantiate() => new GameObject();

    [Test]
    public void Equals_GivenSameTypeAndInstance_ReturnsTrue()
    {
        var instance = Instantiate();

        var handle1 = new InstanceHandle(Type, instance);
        var handle2 = new InstanceHandle(Type, instance);

        Assert.IsTrue(handle1.Equals(handle2));
    }

    [Test]
    public void Equals_GivenDifferentTypeOrInstance_ReturnsFalse()
    {
        var instance1 = Instantiate();
        var instance2 = Instantiate();

        var handle1 = new InstanceHandle(Type, instance1);
        var handle2 = new InstanceHandle(Type, instance2);

        Assert.IsFalse(handle1.Equals(handle2));
    }

    [Test]
    public void IsStatic_GivenNullInstance_ReturnsTrue()
    {
        var handle = new InstanceHandle(Type, null);

        Assert.IsTrue(handle.IsStatic);
    }

    [Test]
    public void Valid_GivenNotNullTypeAndNullInstance_ReturnsTrue()
    {
        var handle = new InstanceHandle(Type, null);

        Assert.IsTrue(handle.Valid);
    }

    [Test]
    public void Valid_GivenNullTypeAndNotNullInstance_ReturnsFalse()
    {
        var instance = Instantiate();

        var handle = new InstanceHandle(null, instance);

        Assert.IsFalse(handle.Valid);
    }

    [Test]
    public void Type_GivenType_ReturnsProperties()
    {
        var instance = Instantiate();
        var instanceId = instance.GetInstanceID();

        var handle = new InstanceHandle(Type, instance);

        Assert.AreEqual(instanceId, handle.InstanceId);
        Assert.AreEqual(Type, handle.Type);
        Assert.AreEqual(instance, handle.Instance);
    }
}
#endif //OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
