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
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.Build;
using UnityEngine;
using UnityEngine.TestTools;

internal class OVRProjectSetupTests : OVRPluginEditModeTest
{
    private const string TaskMessage = "Test";

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        OVRProjectSetup.SetupTemporaryRegistry();
        OVRConfigurationTaskSourceCode.Mock();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        OVRProjectSetup.RestoreRegistry();
        OVRConfigurationTaskSourceCode.Unmock();

        yield return base.UnityTearDown();
    }

    [UnityTest]
    public IEnumerator TestProjectSetupSummaryUpdate()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            null);

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, false.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));

        LogAssert.Expect(LogType.Warning,
            OVRConfigurationTaskUpdaterSummary.GetFullLogMessage(OVRProjectSetup.TaskLevel.Required, 1));

        OVRConfigurationTaskUpdaterSummary taskSummary = null;
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Summary,
            onCompleted: processor => taskSummary = (processor as OVRConfigurationTaskUpdater)?.Summary);
        Assert.AreEqual(true, taskSummary.HasAvailableFixes);
        Assert.AreEqual(true, taskSummary.HasFixes(OVRProjectSetup.TaskLevel.Required));
        Assert.AreEqual(false, taskSummary.HasFixes(OVRProjectSetup.TaskLevel.Recommended));
        Assert.AreEqual(1, taskSummary.GetNumberOfFixes(OVRProjectSetup.TaskLevel.Required));
        Assert.AreEqual(0, taskSummary.GetNumberOfFixes(OVRProjectSetup.TaskLevel.Recommended));
        Assert.AreEqual(OVRProjectSetup.TaskLevel.Required, taskSummary.HighestFixLevel);

        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestProjectSetupAddDuplicateTask()
    {
        var targetGroup = BuildTargetGroup.Android;
        var task1 = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            targetGroup,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            true);

        var task2 = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            targetGroup,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            true);

        OVRProjectSetup.AddTask(task1);
        OVRProjectSetup.AddTask(task2);

        var tasks = OVRProjectSetup.GetTasks(targetGroup, false);

        Assert.AreEqual(1, tasks.Count());

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestProjectSetupLogError()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            true);

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, false.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));
        LogAssert.Expect(LogType.Warning, task.GetFullLogMessage(target));
        LogAssert.Expect(LogType.Warning,
            OVRConfigurationTaskUpdaterSummary.GetFullLogMessage(OVRProjectSetup.TaskLevel.Required, 1));
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Changed);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestProjectSetupLogConditionalLevelError()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Unknown,
            buildTargetGroup => false, // Not Done
            buildTargetGroup => { },
            (Func<BuildTargetGroup, OVRProjectSetup.TaskLevel>)(buildTargetGroup =>
                (buildTargetGroup == BuildTargetGroup.Android)
                    ? OVRProjectSetup.TaskLevel.Required
                    : OVRProjectSetup.TaskLevel.Optional),
            TaskMessage,
            null,
            null,
            true);

        OVRProjectSetup.AddTask(task);

        // Test for Android
        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, false.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));
        LogAssert.Expect(LogType.Warning, task.GetFullLogMessage(target));
        LogAssert.Expect(LogType.Warning,
            OVRConfigurationTaskUpdaterSummary.GetFullLogMessage(OVRProjectSetup.TaskLevel.Required, 1));
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Changed);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);

        // Test for Standalone, no asserts expected because the level is optional
        target = BuildTargetGroup.Standalone;
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, false.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));
        LogAssert.NoUnexpectedReceived();
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Changed);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestProjectSetupLogNoError()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            true);

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, false.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));
        LogAssert.NoUnexpectedReceived();
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Disabled);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestProjectSetupLogWarning()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Recommended,
            TaskMessage,
            null,
            null,
            true);

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, false.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));
        LogAssert.Expect(LogType.Warning,
            OVRConfigurationTaskUpdaterSummary.GetFullLogMessage(OVRProjectSetup.TaskLevel.Recommended, 1));
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Changed);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestProjectSetupLogNoMessage()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Optional,
            TaskMessage,
            null,
            null,
            true);

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, false.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));
        LogAssert.Expect(LogType.Log,
            OVRConfigurationTaskUpdaterSummary.GetFullLogMessage(OVRProjectSetup.TaskLevel.Optional, 1));
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Changed);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestIgnoredTaskError()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            true);

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, true);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, true.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));

        // We expect no error, because the task is being ignored
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Changed);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestInvalidTaskError()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false, // Not done
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            false); // Invalid

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option);

        // We expect no error, because the task is being ignored
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Changed);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestSettingItem()
    {
        const string key = "OVRProjectSetupTests.TestSettingItem";
        OVRProjectSetupSettingBool settingItem = new OVRProjectSetupUserSettingBool(key, false);
        settingItem.Value = true;
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, key)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, true.ToString());
        Assert.AreEqual(true, settingItem.Value);

        OVRProjectSetupSettingBool otherSettingItem = new OVRProjectSetupUserSettingBool(key, false);
        Assert.AreEqual(true, otherSettingItem.Value);
        Assert.AreEqual(settingItem.Value, otherSettingItem.Value);

        settingItem = new OVRProjectSetupProjectSettingBool(key, false);
        settingItem.Value = true;
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, key)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, true.ToString());
        Assert.AreEqual(true, settingItem.Value);

        otherSettingItem = new OVRProjectSetupProjectSettingBool(key, false);
        Assert.AreEqual(true, otherSettingItem.Value);
        Assert.AreEqual(settingItem.Value, otherSettingItem.Value);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestSettingItemHasRecordedValue([Values] bool settingValue)
    {
        const string key = "OVRProjectSetupTests.TestSettingItem";

        OVRProjectSetupSettings.GetProjectConfig(create: false)
            ?.RemoveProjectSetupBool($"{OVRProjectSetup.KeyPrefix}.{key}");

        var settingItem = new OVRProjectSettingBool(key, false);

        Assert.IsFalse(settingItem.HasRecordedValue);
        settingItem.Value = settingValue;

        Assert.IsTrue(settingItem.HasRecordedValue);

        var otherSettingItem = new OVRProjectSettingBool(key, false);
        Assert.IsTrue(settingItem.HasRecordedValue);
        Assert.AreEqual(settingValue, otherSettingItem.Value);

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestConstSettingItem()
    {
        const string key = "OVRProjectSetupTests.TestSettingConstItem";
        OVRProjectSetupSettingBool settingItem = new OVRProjectSetupConstSettingBool(key, true);
        // Confirm we get the correct default value
        Assert.AreEqual(true, settingItem.Value);

        // Confirm setter doesn't do anything
        settingItem.Value = false;
        Assert.AreEqual(true, settingItem.Value);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestOnlyOnceSettingItem()
    {
        const string key = "OVRProjectSetupTests.TestSettingOnlyOnceItem";
        var settingItem = new OVRProjectSetupOnlyOnceSettingBool(key);

        // Reset it first
        settingItem.Reset();

        // First time is true
        Assert.AreEqual(true, settingItem.Value);

        // Second time is false
        Assert.AreEqual(false, settingItem.Value);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestOptionalLambdaType()
    {
        // Testing implicit TValueType operator
        OptionalLambdaType<bool, bool> optionalLambdaType = true;

        // Testing Default for TValueType
        Assert.AreEqual(true, optionalLambdaType.Default);

        // Testing GetValue foc TValueType
        Assert.AreEqual(true, optionalLambdaType.GetValue(true));
        Assert.AreEqual(true, optionalLambdaType.GetValue(false));

        // Testing implicit Func<> operator
        optionalLambdaType = (Func<bool, bool>)(parameter => !parameter);

        // Testing Default for Func<>
        Assert.AreEqual(true, optionalLambdaType.Default);

        // Testing GetValue for Func<>
        Assert.AreEqual(false, optionalLambdaType.GetValue(true));
        Assert.AreEqual(true, optionalLambdaType.GetValue(false));

        // Testing Uncached
        int value = 0;
        OptionalLambdaType<bool, int> uncachedOptionalLambda =
            new OptionalLambdaTypeWithLambda<bool, int>((Func<bool, int>)(boolean => value++));

        // Expecting 0
        Assert.AreEqual(0, uncachedOptionalLambda.GetValue(true));

        // Value is now 1,we're expecting 1
        Assert.AreEqual(1, uncachedOptionalLambda.GetValue(true));

        // Testing Cache
        value = 0;
        OptionalLambdaType<bool, int> cachedOptionalLambda =
            new OptionalLambdaTypeWithCachedLambda<bool, int>((Func<bool, int>)(boolean => value++));

        // Expecting 0
        Assert.AreEqual(0, cachedOptionalLambda.GetValue(true));

        // Value is now 1, but because of cache, we're expecting 0
        Assert.AreEqual(0, cachedOptionalLambda.GetValue(true));

        cachedOptionalLambda.InvalidateCache(true);

        // Value is still 1, but we've invalidated cache, we're expecting 1
        Assert.AreEqual(1, cachedOptionalLambda.GetValue(true));

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestBuildValidator()
    {
        var level = OVRProjectSetup.TaskLevel.Required;
        var done = false;

        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            (Func<BuildTargetGroup, bool>)(buildTargetGroup => done),
            buildTargetGroup => { },
            (Func<BuildTargetGroup, OVRProjectSetup.TaskLevel>)(buildTargetGroup => level),
            TaskMessage,
            null,
            null,
            true);

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, false.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));
        var message = task.GetFullLogMessage(target);

        // Testing failing task
        TestPreprocessBuild(true, message, target, true);

        // Testing ignored task
        task.SetIgnored(target, true);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, true.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));
        TestPreprocessBuild(false, message, target, false);
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, false.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, task.ComputeIgnoreUid(target));

        // Testing completed task
        done = true;
        TestPreprocessBuild(false, message, target, true);
        done = false;

        // Testing low level task
        level = OVRProjectSetup.TaskLevel.Optional;
        TestPreprocessBuild(false, message, target, true);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    private void TestPreprocessBuild(bool expectFailure, string expectedLog, BuildTargetGroup target, bool expectChange)
    {
        TestPreprocessBuildCase(expectFailure, OVRProjectSetup.RequiredThrowErrors.Value, expectedLog, target);
        if (expectChange)
        {
            OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);
        }
    }

    private void TestPreprocessBuildCase(bool expectedFailure, bool expectException, string expectedLog,
        BuildTargetGroup target)
    {
        if (!expectedFailure)
        {
            OVRProjectSetupBuildValidator.PreprocessBuild(target);
            return;
        }

        if (expectException)
        {
            try
            {
                OVRProjectSetupBuildValidator.PreprocessBuild(target);
            }
            catch (BuildFailedException e)
            {
                Assert.AreEqual(e.Message, expectedLog);
            }

            return;
        }

        LogAssert.Expect(LogType.Warning, expectedLog);
        OVRProjectSetupBuildValidator.PreprocessBuild(target);
    }

    [UnityTest]
    public IEnumerator TestGoToSource()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false, // Not done
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            false); // Invalid

        Assert.AreEqual(true, task.SourceCode.Open());
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.GoToSource);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestProjectConfig()
    {
        const string key = "TestProjectConfig";
        const bool defaultValue = false;
        var testSetting = new OVRProjectSetupProjectSettingBool(key, defaultValue);

        // Getting current value
        var value = testSetting.Value;

        var projectConfig = OVRProjectSetupSettings.GetProjectConfig(refresh: true, create: true);
        // Testing lazy internal getter of the Setting Item
        Assert.AreEqual(value, testSetting.Value);

        // Swapping/Setting new value
        value = !value;
        testSetting.Value = value;
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, value.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Uid, key);

        // Testing setter, directly from the project Config
        Assert.AreEqual(value, projectConfig.GetProjectSetupBool(testSetting.Key, testSetting.Default));

        // Testing lazy internal getter of the Setting Item
        Assert.AreEqual(value, testSetting.Value);

        var lazyProjectConfig = OVRProjectSetupSettings.GetProjectConfig(refresh: false, create: false);
        // Confirming that lazy provides the same config
        Assert.AreEqual(lazyProjectConfig, projectConfig);
        // Testing directly from the lazy project Config
        Assert.AreEqual(value, lazyProjectConfig.GetProjectSetupBool(testSetting.Key, testSetting.Default));

        var refreshedProjectConfig = OVRProjectSetupSettings.GetProjectConfig(refresh: true, create: false);
        // Confirming that refresh actually provides the same config
        Assert.AreEqual(refreshedProjectConfig, projectConfig);
        // Testing directly from the refreshed project config
        Assert.AreEqual(value, refreshedProjectConfig.GetProjectSetupBool(testSetting.Key, testSetting.Default));

        // Also testing the OVRProjectConfig cache
        var ovrProjectConfig = OVRProjectConfig.GetProjectConfig();
        var cachedOvrProjectConfig = OVRProjectConfig.CachedProjectConfig;
        Assert.AreEqual(ovrProjectConfig, cachedOvrProjectConfig);

        OVRTelemetry.TestExpectations();

        yield return null;
    }

    [UnityTest]
    public IEnumerator TestInteractionFlow()
    {
        OVRProjectSetupSettingsProvider.ResetInteraction();

        const double timeSpent = 0.2;

        var level = OVRProjectSetup.TaskLevel.Required;
        var target = BuildTargetGroup.Android;

        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            target,
            (Func<BuildTargetGroup, bool>)(buildTargetGroup => false),
            buildTargetGroup => { },
            (Func<BuildTargetGroup, OVRProjectSetup.TaskLevel>)(buildTargetGroup => level),
            TaskMessage,
            null,
            null,
            true);

        // Add a deficient task
        OVRProjectSetup.AddTask(task);
        task.SetIgnored(target, false);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option);
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Changed);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Summary);

        // Open UPST
        // Because batchmode doesn't have Editor UI, we have to fake the UI interactions
        var provider =
            OVRProjectSetupSettingsProvider
                .CreateProjectValidationSettingsProvider() as OVRProjectSetupSettingsProvider;
        provider?.OnActivate(null, null);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Open);

        // Spend some time on the UPST
        OVRProjectSetupSettingsProvider.OpenTimestamp -= timeSpent;

        // Resolve the problem by ignoring the task
        task.SetIgnored(target, true);
        OVRProjectSetupSettingsProvider.SetNewInteraction(OVRProjectSetupSettingsProvider.Interaction.Ignored);
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Option);
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Changed);

        // Close UPST
        provider?.OnDeactivate();
        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.Close)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Interaction,
                OVRProjectSetupSettingsProvider.Interaction.Ignored.ToString());

        OVRTelemetry.Expect(OVRProjectSetupTelemetryEvent.EventTypes.InteractionFlow)
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Interaction,
                OVRProjectSetupSettingsProvider.Interaction.Ignored.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.Value, 1.ToString())
            .AddAnnotation(OVRProjectSetupTelemetryEvent.AnnotationTypes.ValueAfter, 0.ToString());

        OVRTelemetry.TestExpectations();

        yield return null;
    }

#if UNITY_XR_CORE_UTILS
    [UnityTest]
    public IEnumerator TestProjectValidationToolMirror()
    {
        var isFixed = false;
        var level = OVRProjectSetup.TaskLevel.Required;
        var target = BuildTargetGroup.Android;
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            target,
            (Func<BuildTargetGroup, bool>)(buildTargetGroup => isFixed),
            buildTargetGroup => { isFixed = true;},
            (Func<BuildTargetGroup, OVRProjectSetup.TaskLevel>)(buildTargetGroup => level),
            TaskMessage,
            null,
            null,
            true);

        // Add a deficient task
        OVRProjectSetup.AddTask(task);
        task.SetIgnored(target, false);

        // We'd want to make sure that the rule is added here
        // but there is no public method to get the Validation rules for the moment
        var mirrorRule = task.ToValidationRule(target);
        Assert.That(mirrorRule.Message, Is.EqualTo(task.Message.GetValue(target)));
        Assert.That(mirrorRule.IsRuleEnabled.Invoke(), Is.EqualTo(task.Valid.GetValue(target)));
        Assert.That(mirrorRule.Error, Is.True);

        Unity.XR.CoreUtils.Editor.BuildValidator.FixIssues(
            new List<Unity.XR.CoreUtils.Editor.BuildValidationRule>() { mirrorRule }, "Unit Test");

        Assert.That(isFixed, Is.True);
        yield return null;
    }

    [UnityTest]
    public IEnumerator TestInvalidUnknownTargetMirrorRule()
    {
        // Unknown target should always fail to generate a ValidationRule
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Unknown,
            (Func<BuildTargetGroup, bool>)(buildTargetGroup => false),
            buildTargetGroup => { },
            (Func<BuildTargetGroup, OVRProjectSetup.TaskLevel>)(buildTargetGroup => OVRProjectSetup.TaskLevel.Required),
            TaskMessage,
            null,
            null,
            true);

        var brokenMirrorRule = task.ToValidationRule(BuildTargetGroup.Unknown);
        Assert.IsNull(brokenMirrorRule);
        yield return null;
    }

    [UnityTest]
    public IEnumerator TestInvalidNoFixMirrorRule()
    {
        // Tasks with no fix should always fail to generate a ValidationRule
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Unknown,
            (Func<BuildTargetGroup, bool>)(buildTargetGroup => false),
            null,
            (Func<BuildTargetGroup, OVRProjectSetup.TaskLevel>)(buildTargetGroup => OVRProjectSetup.TaskLevel.Required),
            TaskMessage,
            null,
            null,
            true);

        var brokenMirrorRule = task.ToValidationRule(BuildTargetGroup.Android);
        Assert.IsNull(brokenMirrorRule);
        yield return null;
    }
#endif

}

#endif // OVRPLUGIN_TESTING
