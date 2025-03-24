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

using System.Collections;
using Meta.XR.ImmersiveDebugger;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

internal class DebugInspectorTests_TestMonoBehaviourA : MonoBehaviour
{
    public float testFloat;
}

internal class DebugInspectorTests_TestMonoBehaviourB : MonoBehaviour
{
    public int testInt;
}

internal class DebugInspectorTests : OVRPluginPlayModeTest
{
    private GameObject _gameObject;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        OVRTelemetry.ResetExpectations();
        _gameObject = new GameObject();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        GameObject.Destroy(_gameObject);
        DebugInspectorManager.Destroy();
        OVRTelemetry.TestExpectations();
        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestParsing()
    {
        var transform = _gameObject.transform;
        var testMonoBehaviourA = _gameObject.AddComponent<DebugInspectorTests_TestMonoBehaviourA>();
        var debugInspector = _gameObject.AddComponent<DebugInspector>();
        yield return null;

        Validate(debugInspector, new Component[]{transform, testMonoBehaviourA});

        _gameObject.SetActive(false);
        Object.Destroy(testMonoBehaviourA);
        yield return null;

        var testMonoBehaviourB = _gameObject.AddComponent<DebugInspectorTests_TestMonoBehaviourB>();
        _gameObject.SetActive(true);
        yield return null;

        Validate(debugInspector, new Component[]{transform, null, testMonoBehaviourB});

        _gameObject.SetActive(false);
        testMonoBehaviourA = _gameObject.AddComponent<DebugInspectorTests_TestMonoBehaviourA>();
        _gameObject.SetActive(true);
        yield return null;

        Validate(debugInspector, new Component[]{transform, testMonoBehaviourA, testMonoBehaviourB});
    }

    private void Validate(DebugInspector debugInspector, Component[] components)
    {
        var count = components.Length;
        Assert.That(debugInspector.Registry.Handles.Count, Is.EqualTo(count));
        for(var i=0; i<count; i++)
        {
            var component = components[i];
            var handle = debugInspector.Registry.Handles[i];
            if (component == null)
            {
                Assert.False(handle.Valid);
            }
            else
            {
                Assert.True(handle.Valid);
                Assert.That(handle.Type, Is.EqualTo(component.GetType()));
                var instanceHandle = handle.InstanceHandle;
                Assert.True(instanceHandle.Valid);
                Assert.That(instanceHandle.Instance, Is.EqualTo(component));
                Assert.That(handle.inspectedMembers.Count, Is.GreaterThan(0));
                var member = handle.inspectedMembers[0];
                Assert.True(member.Valid);
            }
        }
    }
}
#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
