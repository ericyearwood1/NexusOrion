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

using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

internal class OVRConfigurationTaskProcessorTests : OVRPluginEditModeTest
{
    private List<bool> _tasksDone;
    private List<OVRConfigurationTask> _tasks;
    private Dictionary<OVRConfigurationTaskProcessor, bool> _callbackFlags;

    private const BuildTargetGroup Target = BuildTargetGroup.Android;
    private const int TimeoutInMs = 10000;
    private const int NumberOfTasks = 3;

    private int _slowUpdateDuration;
    private int _slowFixDuration;

    private OVRConfigurationTaskRegistry _registry;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        yield return new WaitUntil(() => OVRProjectSetup.IsReady);
        OVRProjectSetup.SetupTemporaryRegistry();
        _registry = new OVRConfigurationTaskRegistry();
        var dummyUpdateProcessor =
            new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled, false, null);
        _slowUpdateDuration = dummyUpdateProcessor.AllocatedTimeInMs + 10;
        var dummyFixerProcessor =
            new OVRConfigurationTaskFixer(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled, false, null);
        _slowFixDuration = dummyFixerProcessor.AllocatedTimeInMs + 10;
        _tasksDone = new List<bool>(NumberOfTasks);
        _tasks = new List<OVRConfigurationTask>(NumberOfTasks);
        _callbackFlags = new Dictionary<OVRConfigurationTaskProcessor, bool>();
        for (var i = 0; i < NumberOfTasks; i++)
        {
            var index = i;
            _tasksDone.Add(false);
            var task = new OVRConfigurationTask(OVRProjectSetup.TaskGroup.Compatibility,
                BuildTargetGroup.Android,
                buildTargetGroup =>
                {
                    System.Threading.Thread.Sleep(_slowUpdateDuration);
                    return _tasksDone[index];
                },
                buildTargetGroup =>
                {
                    System.Threading.Thread.Sleep(_slowFixDuration);
                    _tasksDone[index] = true;
                },
                OVRProjectSetup.TaskLevel.Required,
                "Test" + index,
                null,
                null,
                true);
            _tasks.Add(task);
            _registry.AddTask(task);
        }

        yield return null;
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        OVRProjectSetup.RestoreRegistry();
        yield return new WaitUntil(() => OVRProjectSetup.IsReady);
        yield return base.UnityTearDown();
    }

    private void TestTasksAreDone(bool expected)
    {
        foreach (var task in _tasks)
        {
            Assert.AreEqual(expected, task.IsDone(Target));
        }
    }

    private void TestProcessorsAreDone(List<OVRConfigurationTaskProcessor> processors, bool expected)
    {
        foreach (var processor in processors)
        {
            Assert.AreEqual(expected, processor.Completed);
            _callbackFlags.TryGetValue(processor, out var callbacked);
            Assert.AreEqual(expected, callbacked);
        }
    }

    private void Callback(OVRConfigurationTaskProcessor processor)
    {
        _callbackFlags[processor] = true;
    }

    private void CallbackUpdate(OVRConfigurationTaskProcessor processor, bool expectOutstandingTasks)
    {
        Assert.AreEqual(processor.Type, OVRConfigurationTaskProcessor.ProcessorType.Updater);
        var updater = processor as OVRConfigurationTaskUpdater;
        Assert.NotNull(updater);
        Assert.AreEqual(expectOutstandingTasks, updater.Summary.HasAvailableFixes);
    }

    private void CallbackFix(OVRConfigurationTaskProcessor processor)
    {
        Callback(processor);
        Assert.AreEqual(processor.Type, OVRConfigurationTaskProcessor.ProcessorType.Fixer);
        var fixer = processor as OVRConfigurationTaskFixer;
        Assert.NotNull(fixer);
    }

    private void CallbackUpdateExpectOutstandingTasks(OVRConfigurationTaskProcessor processor)
    {
        Callback(processor);
        CallbackUpdate(processor, true);
    }

    private void CallbackUpdateExpectNoOutstandingTasks(OVRConfigurationTaskProcessor processor)
    {
        Callback(processor);
        CallbackUpdate(processor, false);
    }

    private IEnumerator Test(List<OVRConfigurationTaskProcessor> processors)
    {
        bool willFix = false;
        bool willBlock = false;
        int expectedFrameCounter = 0;
        int frameCounter = 0;
        var queue = new OVRConfigurationTaskProcessorQueue();

        // Tasks are not done
        TestTasksAreDone(false);

        // Processors are not completed
        TestProcessorsAreDone(processors, false);

        foreach (var processor in processors)
        {
            if (processor.Type == OVRConfigurationTaskProcessor.ProcessorType.Fixer)
            {
                willFix = true;
                // Expect Fix or Update for individual tasks
                foreach (var task in _tasks)
                {
                    OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Fix)
                        .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.Uid.ToString());
                }
            }
            else
            {
                if (!willFix)
                {
                    OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);
                }
            }

            if (processor.Blocking)
            {
                willBlock = true;
            }
            else
            {
                expectedFrameCounter += NumberOfTasks;
            }

            // Request the Processor
            queue.Request(processor);
        }

        if (!willBlock)
        {
            expectedFrameCounter++;

            // Processors are still not completed
            TestProcessorsAreDone(processors, false);

            yield return new WaitWhile(() =>
            {
                frameCounter++;
                return queue.Busy;
            });
        }
        else
        {
            expectedFrameCounter = 0;
        }


        Assert.AreEqual(expectedFrameCounter, frameCounter);

        // Processors are now completed
        TestProcessorsAreDone(processors, true);

        TestTasksAreDone(willFix);

        // Queue is nor blocked nor busy
        Assert.AreEqual(false, queue.Busy);
        Assert.AreEqual(false, queue.Blocked);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestQueueFixBlocking()
    {
        var processors = new List<OVRConfigurationTaskProcessor>();
        // Create the Fix Processor (Blocking)
        processors.Add(new OVRConfigurationTaskFixer(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            true, CallbackFix));
        yield return Test(processors);
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestQueueFixNonBlocking()
    {
        var processors = new List<OVRConfigurationTaskProcessor>();
        // Create the Fix Processor (Blocking)
        processors.Add(new OVRConfigurationTaskFixer(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            false, CallbackFix));
        yield return Test(processors);
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestQueueUpdateBlocking()
    {
        var processors = new List<OVRConfigurationTaskProcessor>();
        // Create the Fix Processor (Blocking)
        processors.Add(new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            true, CallbackUpdateExpectOutstandingTasks));
        yield return Test(processors);
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestQueueUpdateNonBlocking()
    {
        var processors = new List<OVRConfigurationTaskProcessor>();
        // Create the Fix Processor (Blocking)
        processors.Add(new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            false, CallbackUpdateExpectOutstandingTasks));
        yield return Test(processors);
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestQueueMultipleBlocking()
    {
        var processors = new List<OVRConfigurationTaskProcessor>();
        // Create the Fix Processor (Blocking)
        processors.Add(new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            false, CallbackUpdateExpectOutstandingTasks));
        processors.Add(new OVRConfigurationTaskFixer(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            false, CallbackFix));
        processors.Add(new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            true, CallbackUpdateExpectNoOutstandingTasks));
        yield return Test(processors);
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestQueueMultipleNonBlocking()
    {
        var processors = new List<OVRConfigurationTaskProcessor>();
        // Create the Fix Processor (Blocking)
        processors.Add(new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            false, CallbackUpdateExpectOutstandingTasks));
        processors.Add(new OVRConfigurationTaskFixer(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            false, CallbackFix));
        processors.Add(new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
            false, CallbackUpdateExpectNoOutstandingTasks));
        yield return Test(processors);
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestQueueMultipleBlockingQueuingAnotherProcessor()
    {
        var queue = new OVRConfigurationTaskProcessorQueue();

        queue.Request(
            new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
                true,
                (processorA) => queue.Request(new OVRConfigurationTaskFixer(_registry, Target, null,
                    OVRProjectSetup.LogMessages.Disabled, true,
                    (processorB) => queue.Request(new OVRConfigurationTaskUpdater(_registry, Target, null,
                        OVRProjectSetup.LogMessages.Disabled, true, null))))));

        Assert.AreEqual(false, queue.Busy);
        Assert.AreEqual(false, queue.Blocked);

        yield return null;
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestQueueUpdateRegisterBlocking()
    {
        var queue = new OVRConfigurationTaskProcessorQueue();

        Assert.AreEqual(false, IsUpdateRegistered(queue));

        queue.Request(
            new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
                true,
                (processorA) => queue.Request(new OVRConfigurationTaskFixer(_registry, Target, null,
                    OVRProjectSetup.LogMessages.Disabled, true,
                    (processorB) => queue.Request(new OVRConfigurationTaskUpdater(_registry, Target, null,
                        OVRProjectSetup.LogMessages.Disabled, true, null))))));

        Assert.AreEqual(false, queue.Busy);
        Assert.AreEqual(false, queue.Blocked);

        Assert.AreEqual(false, IsUpdateRegistered(queue));

        yield return null;
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestQueueUpdateRegisterNonBlocking()
    {
        var queue = new OVRConfigurationTaskProcessorQueue();

        Assert.AreEqual(false, IsUpdateRegistered(queue));

        queue.Request(
            new OVRConfigurationTaskUpdater(_registry, Target, null, OVRProjectSetup.LogMessages.Disabled,
                false,
                (processorA) => queue.Request(new OVRConfigurationTaskFixer(_registry, Target, null,
                    OVRProjectSetup.LogMessages.Disabled, false,
                    (processorB) => queue.Request(new OVRConfigurationTaskUpdater(_registry, Target, null,
                        OVRProjectSetup.LogMessages.Disabled, false, null))))));

        yield return new WaitWhile(() => queue.Busy);
        Assert.AreEqual(false, queue.Blocked);
        Assert.AreEqual(false, IsUpdateRegistered(queue));

        yield return null;
    }

    private bool IsUpdateRegistered(OVRConfigurationTaskProcessorQueue queue)
    {
        var method = queue.GetType().GetMethod("Update", BindingFlags.Instance | BindingFlags.NonPublic);

        foreach (var callback in EditorApplication.update.GetInvocationList())
        {
            if (callback.Method == method && callback.Target == queue)
            {
                return true;
            }
        }

        return false;
    }
}

#endif // OVRPLUGIN_TESTING
