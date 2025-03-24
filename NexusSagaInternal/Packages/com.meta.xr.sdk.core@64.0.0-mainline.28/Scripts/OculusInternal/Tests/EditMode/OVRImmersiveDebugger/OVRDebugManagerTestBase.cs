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
using Meta.XR.ImmersiveDebugger.Manager;
using Meta.XR.ImmersiveDebugger.UserInterface;
using Meta.XR.ImmersiveDebugger.Utils;
using System.Collections;
using NUnit.Framework;
using UnityEngine;
using Object = UnityEngine.Object;

internal class OVRDebugManagerTestBase<TestClassType, ManagerType> : OVRPluginEditModeTest
    where TestClassType : MonoBehaviour
    where ManagerType : IDebugManager, new()
{
    protected readonly IDebugUIPanel Panel = new GameObject().AddComponent<MockUIPanel>();
    protected const int NumberOfInstances = 3;
    protected readonly TestClassType[] Instances = new TestClassType[3];
    protected ManagerType Manager;
    protected InstanceCache Cache;

    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        Manager = new ManagerType();
        Cache = new InstanceCache();
        Manager.Setup(Panel, Cache);
        Cache.RegisterClassType(typeof(TestClassType));
        var inspector = Panel.RegisterInspector(new InstanceHandle(typeof(TestClassType), null));
        Assert.IsNotNull(inspector);
        for(var i = 0; i<NumberOfInstances; i++)
        {
            Instances[i] = new GameObject().AddComponent<TestClassType>();
            var instanceInspector = Panel.RegisterInspector(new InstanceHandle(typeof(TestClassType), Instances[i]));
            Assert.IsNotNull(instanceInspector);
        }
        Cache.RetrieveInstances();
    }

    public override IEnumerator UnityTearDown()
    {
        foreach (var instance in Instances)
        {
            Panel.UnregisterInspector(new InstanceHandle(typeof(TestClassType), instance));
            Object.DestroyImmediate(instance.gameObject);
        }
        Panel.UnregisterInspector(new InstanceHandle(typeof(TestClassType), null));
        Cache = null;
        yield return base.UnityTearDown();
    }

    public IEnumerator ProcessType()
    {
        var type = typeof(TestClassType);

        Manager.ProcessType(type);

        yield return null;
    }

    protected IInspector GetInspector(TestClassType instance)
    {
        Type type = typeof(TestClassType);
        var inspector = Panel.GetInspector(new InstanceHandle(type, instance));
        Assert.IsNotNull(inspector);
        return inspector;
    }

    protected IMember GetMember(TestClassType instance, string memberName)
    {
        var inspector = GetInspector(instance);
        Type type = typeof(TestClassType);
        var memberInfo = type.GetMember(memberName)[0];
        var member = inspector.GetMember(memberInfo);
        Assert.IsNotNull(member);
        return member;
    }
}
#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
