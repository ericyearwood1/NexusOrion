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
using Meta.XR.ImmersiveDebugger.DebugData;
using Meta.XR.ImmersiveDebugger.Manager;
using Meta.XR.ImmersiveDebugger.UserInterface;
using Meta.XR.ImmersiveDebugger.Utils;
using System.Collections;
using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Assert = UnityEngine.Assertions.Assert;

internal class DummySubManager : IDebugManager
{
    public readonly List<string> ProcessedTypeNames = new List<string>();
    private string _telemetryAnnotation;

    public DummySubManager(string telemetryAnnotation)
    {
        _telemetryAnnotation = telemetryAnnotation;
    }

    public void Setup(IDebugUIPanel panel, InstanceCache cache)
    {
        throw new NotImplementedException();
    }

    public void ProcessType(Type type)
    {
        ProcessedTypeNames.Add(type.Name);
    }

    public void ProcessTypeFromInspector(Type type, InstanceHandle handle, MemberInfo memberInfo, DebugMember memberAttribute)
    {
        throw new NotImplementedException();
    }

    public string TelemetryAnnotation => _telemetryAnnotation;
    public int GetCountPerType(Type type) => ProcessedTypeNames.Contains(type.Name) ? 1 : 0;
}

internal class TestDebugManager : DebugManager
{
    public readonly DummySubManager DummyManager = new DummySubManager("Dummy");
    public readonly DummySubManager AnotherDummyManager = new DummySubManager("AnotherDummy");
    protected override void InitSubManagers()
    {
        _subDebugManagers.Add(DummyManager);
        _subDebugManagers.Add(AnotherDummyManager);
    }

    public InstanceCache GetInstanceCache()
    {
        return _instanceCache;
    }
}

// A couple of classes annotated
internal class TestClassA : MonoBehaviour
{
    [DebugMember] private float _testFloat;
}

internal class TestClassB : MonoBehaviour
{
    [DebugMember] private float _testFloat;
}

internal class TestClassC : MonoBehaviour
{
    [DebugMember] private float _testFloat;
}

internal class OVRDebugManagerTests : OVRPluginPlayModeTest
{
    private TestDebugManager _debugManager;
    private GameObject _gameObject;
    private MockUIPanel _uiPanel;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        OVRTelemetry.ResetExpectations();
        RuntimeSettings.UseMockInstance = true;
        AssemblyParser.Mock(this.GetType().Assembly);
        AssemblyParser.Refresh();
        yield return new WaitUntil(() => AssemblyParser.Ready);
        _gameObject = new GameObject("obj");
        _uiPanel = _gameObject.AddComponent<MockUIPanel>();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        UnityEngine.Object.DestroyImmediate(_gameObject);
        AssemblyParser.Unmock();
        OVRTelemetry.TestExpectations();
        yield return base.UnityTearDown();
    }

    private int GetTypeWithValidInstancesCount()
    {
        int typeWithValidInstances = 0;
        foreach ((var type, var instances) in _debugManager.GetInstanceCache().CacheData)
        {
            if (instances.Count != 0)
            {
                typeWithValidInstances++;
            }
        }
        return typeWithValidInstances;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestStart()
    {
        _gameObject.AddComponent<TestClassA>();
        _gameObject.AddComponent<TestClassB>();
        _gameObject.AddComponent<TestClassC>();
        _debugManager = _gameObject.AddComponent<TestDebugManager>();
        yield return null;
        ExpectTelemetryRun();
        ExpectTelemetryComponentTracked(typeof(TestClassA), Telemetry.State.OnStart, 1);
        ExpectTelemetryComponentTracked(typeof(TestClassB), Telemetry.State.OnStart, 1);
        ExpectTelemetryComponentTracked(typeof(TestClassC), Telemetry.State.OnStart, 1);
        Assert.AreEqual(GetTypeWithValidInstancesCount(), 3);
        var processedTypeNames = _debugManager.DummyManager.ProcessedTypeNames;
        Assert.IsTrue(processedTypeNames.Contains("TestClassA"));
        Assert.IsTrue(processedTypeNames.Contains("TestClassB"));
        Assert.IsTrue(processedTypeNames.Contains("TestClassC"));
        var anotherProcessedTypeNames = _debugManager.AnotherDummyManager.ProcessedTypeNames;
        Assert.IsTrue(anotherProcessedTypeNames.Contains("TestClassA"));
        Assert.IsTrue(anotherProcessedTypeNames.Contains("TestClassB"));
        Assert.IsTrue(anotherProcessedTypeNames.Contains("TestClassC"));
        ExpectTelemetryComponentTracked(typeof(TestClassA), Telemetry.State.OnDisable, 1);
        ExpectTelemetryComponentTracked(typeof(TestClassB), Telemetry.State.OnDisable, 1);
        ExpectTelemetryComponentTracked(typeof(TestClassC), Telemetry.State.OnDisable, 1);
    }

    private void ExpectTelemetryRun()
    {
        OVRTelemetry.Expect(Telemetry.MarkerId.Run)
            .AddAnnotation(Telemetry.AnnotationType.Method, Telemetry.Method.Attributes.ToString());
    }

    private void ExpectTelemetryComponentTracked(Type type, Telemetry.State state, int instanceCount)
    {
        OVRTelemetry.Expect(Telemetry.MarkerId.ComponentTracked)
            .AddAnnotation(Telemetry.AnnotationType.State, state.ToString())
            .AddAnnotation(Telemetry.AnnotationType.Type, type.Name)
            .AddAnnotation(Telemetry.AnnotationType.Instances, instanceCount.ToString())
            .AddAnnotation(Telemetry.AnnotationType.Method, Telemetry.Method.Attributes.ToString())
            .AddAnnotation(_debugManager.DummyManager.TelemetryAnnotation, 1.ToString())
            .AddAnnotation(_debugManager.AnotherDummyManager.TelemetryAnnotation, 1.ToString());
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestStartWithMultipleLoadedScene()
    {
        _gameObject.AddComponent<TestClassA>();
        _debugManager = _gameObject.AddComponent<TestDebugManager>();

        // Configure multiple scenes with one unloaded
        var newLoadedScene = SceneManager.CreateScene("New Loaded Scene");
        var newLoadedGameObject = new GameObject("new loaded obj");
        newLoadedGameObject.AddComponent<TestClassB>();
        SceneManager.MoveGameObjectToScene(newLoadedGameObject, newLoadedScene);

        var newUnloadedScene = SceneManager.CreateScene("New Unloaded Scene");
        var newUnloadedGameObject = new GameObject("new unloaded obj");
        newUnloadedGameObject.AddComponent<TestClassC>();
        SceneManager.MoveGameObjectToScene(newUnloadedGameObject, newUnloadedScene);
        SceneManager.UnloadSceneAsync(newUnloadedScene);
        // when a scene is unloaded, the instances are not immediately destroyed
        UnityEngine.Object.DestroyImmediate(newUnloadedGameObject);

        yield return null;
        ExpectTelemetryRun();
        Assert.AreEqual(GetTypeWithValidInstancesCount(), 2);
        var processedTypeNames = _debugManager.DummyManager.ProcessedTypeNames;
        Assert.IsTrue(processedTypeNames.Contains("TestClassA"));
        Assert.IsTrue(processedTypeNames.Contains("TestClassB"));
        Assert.IsFalse(processedTypeNames.Contains("TestClassC"));
        var anotherProcessedTypeNames = _debugManager.AnotherDummyManager.ProcessedTypeNames;
        Assert.IsTrue(anotherProcessedTypeNames.Contains("TestClassA"));
        Assert.IsTrue(anotherProcessedTypeNames.Contains("TestClassB"));
        Assert.IsFalse(anotherProcessedTypeNames.Contains("TestClassC"));

        // tear down for this case
        SceneManager.UnloadSceneAsync(newLoadedScene);

        ExpectTelemetryComponentTracked(typeof(TestClassA), Telemetry.State.OnStart, 1);
        ExpectTelemetryComponentTracked(typeof(TestClassA), Telemetry.State.OnDisable, 1);
        ExpectTelemetryComponentTracked(typeof(TestClassB), Telemetry.State.OnDisable, 1);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestRegisterReturnsInspector()
    {
        _debugManager = _gameObject.AddComponent<TestDebugManager>();

        // Waiting for Initialization of the TestDebugManager
        yield return null;
        ExpectTelemetryRun();

        var componentA = _gameObject.AddComponent<TestClassA>();
        var inspector = _debugManager.RegisterInspector(new InstanceHandle(typeof(TestClassA), componentA));
        Assert.IsNotNull(inspector);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestLateInitUpdate()
    {
        var componentA = _gameObject.AddComponent<TestClassA>();
        _debugManager = _gameObject.AddComponent<TestDebugManager>();
        yield return null;
        ExpectTelemetryRun();
        ExpectTelemetryComponentTracked(typeof(TestClassA), Telemetry.State.OnStart, 1);

        Assert.AreEqual(GetTypeWithValidInstancesCount(), 1);
        Assert.IsNotNull(_uiPanel.GetInspector(new InstanceHandle(typeof(TestClassA), componentA)));

        var componentB = _gameObject.AddComponent<TestClassB>();
        _debugManager.ShouldRetrieveInstances = true;
        yield return null;

        Assert.AreEqual(GetTypeWithValidInstancesCount(), 2);
        var processedTypeNames = _debugManager.DummyManager.ProcessedTypeNames;
        Assert.IsTrue(processedTypeNames.Contains("TestClassB"));
        Assert.IsNotNull(_uiPanel.GetInspector(new InstanceHandle(typeof(TestClassB), componentB)));

        UnityEngine.Object.DestroyImmediate(componentB);
        _debugManager.ShouldRetrieveInstances = true;
        yield return null;

        Assert.AreEqual(GetTypeWithValidInstancesCount(), 1);
        Assert.IsNull(_uiPanel.GetInspector(new InstanceHandle(typeof(TestClassB), componentB)));
        ExpectTelemetryComponentTracked(typeof(TestClassA), Telemetry.State.OnDisable, 1);
    }
}
#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
