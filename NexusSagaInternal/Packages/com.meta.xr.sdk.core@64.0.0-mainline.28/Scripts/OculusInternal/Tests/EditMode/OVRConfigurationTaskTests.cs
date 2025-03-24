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

using System;
using System.Collections;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

internal class OVRConfigurationTaskTests : OVRPluginEditModeTest
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

    private const string TaskMessage = "Test";
    private const string Url = "Url Test";
    private const string FixMessage = "Fix Test";

    [UnityTest]
    public IEnumerator TestTaskCreation()
    {
        Assert.DoesNotThrow(() =>
        {
            var task = new OVRConfigurationTask(
                OVRProjectSetup.TaskGroup.Compatibility,
                BuildTargetGroup.Android,
                buildTargetGroup => true,
                buildTargetGroup => { },
                OVRProjectSetup.TaskLevel.Required,
                TaskMessage,
                FixMessage,
                Url,
                true);
        });

        Assert.Throws<ArgumentException>(() =>
        {
            var task = new OVRConfigurationTask(
                OVRProjectSetup.TaskGroup.All,
                BuildTargetGroup.Android,
                buildTargetGroup => true,
                buildTargetGroup => { },
                OVRProjectSetup.TaskLevel.Required,
                TaskMessage,
                FixMessage,
                Url,
                true);
        });

        Assert.Throws<ArgumentNullException>(() =>
        {
            var task = new OVRConfigurationTask(
                OVRProjectSetup.TaskGroup.Compatibility,
                BuildTargetGroup.Android,
                null,
                buildTargetGroup => { },
                OVRProjectSetup.TaskLevel.Required,
                TaskMessage,
                FixMessage,
                Url,
                true);
        });

        Assert.Throws<ArgumentNullException>(() =>
        {
            var task = new OVRConfigurationTask(
                OVRProjectSetup.TaskGroup.Compatibility,
                BuildTargetGroup.Android,
                buildTargetGroup => false,
                buildTargetGroup => { },
                OVRProjectSetup.TaskLevel.Required,
                "",
                FixMessage,
                Url,
                true);
        });

        Assert.Throws<ArgumentNullException>(() =>
        {
            var task = new OVRConfigurationTask(
                OVRProjectSetup.TaskGroup.Compatibility,
                BuildTargetGroup.Android,
                buildTargetGroup => false,
                buildTargetGroup => { },
                OVRProjectSetup.TaskLevel.Required,
                null,
                FixMessage,
                Url,
                true);
        });

        Assert.Throws<ArgumentNullException>(() =>
        {
            var task = new OVRConfigurationTask(
                OVRProjectSetup.TaskGroup.Compatibility,
                BuildTargetGroup.Android,
                buildTargetGroup => false,
                buildTargetGroup => { },
                null,
                TaskMessage,
                FixMessage,
                Url,
                true);
        });

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestTaskProperties()
    {
        var level = OVRProjectSetup.TaskLevel.Required;
        var group = OVRProjectSetup.TaskGroup.Compatibility;
        Func<BuildTargetGroup, bool> isDone = buildTargetGroup => true;
        Action<BuildTargetGroup> fix = buildTargetGroup => { };
        var platform = BuildTargetGroup.Android;

        var task = new OVRConfigurationTask(
            group,
            platform,
            isDone,
            fix,
            level,
            TaskMessage,
            FixMessage,
            Url,
            true);

        var hash = new Hash128();
        hash.Append(TaskMessage);
        Assert.AreEqual(hash, task.Uid);
        Assert.AreEqual(level, task.Level.Default);
        Assert.AreEqual(group, task.Group);
        Assert.AreEqual(platform, task.Platform);
        Assert.AreEqual(TaskMessage, task.Message.Default);
        Assert.AreEqual(FixMessage, task.FixMessage.Default);
        Assert.AreEqual(Url, task.URL.Default);

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestTaskGetDoneState()
    {
        var counter = 0;

        bool IsDone(BuildTargetGroup buildTargetGroup)
        {
            counter++;
            return true;
        }

        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            IsDone,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            true);

        // Should execute the lamdba while the result has not been cached
        Assert.AreEqual(true, task.IsDone(BuildTargetGroup.Android));
        Assert.AreEqual(1, counter);

        Assert.AreEqual(true, task.IsDone(BuildTargetGroup.Android));
        Assert.AreEqual(2, counter);

        task.UpdateAndGetStateChanged(BuildTargetGroup.Standalone);
        Assert.AreEqual(3, counter);

        Assert.AreEqual(true, task.IsDone(BuildTargetGroup.Android));
        Assert.AreEqual(4, counter);

        task.UpdateAndGetStateChanged(BuildTargetGroup.Android);
        Assert.AreEqual(5, counter);

        // Now the result should be cached and the lamdba should not be called again
        Assert.AreEqual(true, task.IsDone(BuildTargetGroup.Android));
        Assert.AreEqual(5, counter);

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestTaskUpdate()
    {
        var result = true;

        bool IsDone(BuildTargetGroup buildTargetGroup)
        {
            // ReSharper disable once AccessToModifiedClosure
            return result;
        }

        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            IsDone,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            FixMessage,
            Url,
            true);

        // Should update on the first run
        Assert.AreEqual(true, task.UpdateAndGetStateChanged(BuildTargetGroup.Android));

        // Should not update on subsequent runs that dont change the state
        Assert.AreEqual(false, task.UpdateAndGetStateChanged(BuildTargetGroup.Android));

        result = false;

        // Should update on the first run after changing the state
        Assert.AreEqual(true, task.UpdateAndGetStateChanged(BuildTargetGroup.Android));

        // Should not update on subsequent runs that dont change the state
        Assert.AreEqual(false, task.UpdateAndGetStateChanged(BuildTargetGroup.Android));

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestSourceCode()
    {
        var st = new System.Diagnostics.StackTrace(true);
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            group => true,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            FixMessage,
            null,
            true);

        // This one is not expected to work, as we don't call AddTask from the OVRProjectSetup
        Assert.AreEqual(true, task.SourceCode.Valid);

        const int depth = 0;
        var frame = st.GetFrame(depth);
        var line = frame.GetFileLineNumber();
        // We're expecting the two lines to match (+1 because it's the next line)
        Assert.AreEqual(task.SourceCode.Line, line + 1);

        yield return null;
    }
}

#endif // OVRPLUGIN_TESTING
