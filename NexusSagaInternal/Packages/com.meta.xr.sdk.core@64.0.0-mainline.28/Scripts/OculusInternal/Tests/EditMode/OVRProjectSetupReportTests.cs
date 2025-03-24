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
using System.IO;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

[TestFixture]
public class OVRProjectSetupReportTests
{
    private const string TaskMessage = "Test message";
    private const string AnotherTaskMessage = "Another message";
    private string _generatedReportFile;

    [SetUp]
    public void BeforeEach()
    {
        OVRProjectSetup.SetupTemporaryRegistry();
        OVRConfigurationTaskSourceCode.Mock();
    }

    [TearDown]
    public void CleanupAfterEach()
    {
        OVRProjectSetup.RestoreRegistry();
        OVRConfigurationTaskSourceCode.Unmock();
        try
        {
            File.Delete(_generatedReportFile);
        }
        catch (IOException e)
        {
            Console.WriteLine("could not clean up the report file: " + e.Message);
        }
    }

    [Test]
    public void TestProjectSetupSummaryGenerateReportFileExistsDefaultFileName()
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

        OVRConfigurationTaskUpdaterSummary taskSummary = null;
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Summary,
            onCompleted: processor => taskSummary = (processor as OVRConfigurationTaskUpdater)?.Summary);


        const string outputFolder = "./";
        _generatedReportFile = taskSummary.GenerateReport(outputFolder);
        Assert.That(File.Exists(_generatedReportFile));
        Assert.That(_generatedReportFile.StartsWith(Path.Combine(outputFolder,
            OVRProjectSetupReport.ReportDefaultFileNamePrefix)));
    }

    [Test]
    public void TestProjectSetupSummaryGenerateReportFileExistsCustomFileName()
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

        OVRConfigurationTaskUpdaterSummary taskSummary = null;
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Summary,
            onCompleted: processor => taskSummary = (processor as OVRConfigurationTaskUpdater)?.Summary);

        const string customFileName = "custom.json";
        const string outputFolder = "./";
        _generatedReportFile = taskSummary.GenerateReport(outputFolder, customFileName);
        Assert.That(File.Exists(_generatedReportFile));
        Assert.AreEqual(_generatedReportFile, Path.Combine(outputFolder, customFileName));
    }

    [Test]
    public void TestProjectSetupSummaryGenerateReportFileExistsDefaultFolder()
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

        OVRConfigurationTaskUpdaterSummary taskSummary = null;
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Summary,
            onCompleted: processor => taskSummary = (processor as OVRConfigurationTaskUpdater)?.Summary);

        const string customFileName = "custom.json";
        _generatedReportFile = taskSummary.GenerateReport(null, customFileName);
        Assert.That(File.Exists(_generatedReportFile));
        Assert.AreEqual(_generatedReportFile,
            Path.Combine(OVRProjectSetupReport.ReportDefaultOutputPath, customFileName));
    }

    [Test]
    public void TestProjectSetupSummaryGenerateReportStructure()
    {
        var target = BuildTargetGroup.Android;
        var task1 = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            true);
        task1.SetIgnored(target, false);
        OVRProjectSetup.AddTask(task1);
        var task2 = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => false,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            AnotherTaskMessage,
            null,
            null,
            true);
        task2.SetIgnored(target, false);
        OVRProjectSetup.AddTask(task2);

        OVRConfigurationTaskUpdaterSummary taskSummary = null;
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Summary,
            onCompleted: processor => taskSummary = (processor as OVRConfigurationTaskUpdater)?.Summary);

        _generatedReportFile = taskSummary.GenerateReport("./");
        var generatedFileName = Path.GetFileName(_generatedReportFile);
        Assert.That(generatedFileName.StartsWith(OVRProjectSetupReport.ReportDefaultFileNamePrefix));

        string jsonText = System.IO.File.ReadAllText(_generatedReportFile);
        Assert.That(!string.IsNullOrEmpty(jsonText));

        OVRProjectSetupJsonReport report = JsonUtility.FromJson<OVRProjectSetupJsonReport>(jsonText);

        var creationTime = DateTime.Parse(report.createdAt);
        var timeSinceReportCreation = DateTime.Now.ToUniversalTime().Subtract(creationTime);
        Assert.Positive(timeSinceReportCreation.TotalSeconds);
        Assert.AreEqual(2, report.tasksStatus.Count);
        Assert.AreEqual(target.ToString(), report.buildTargetGroup);
        Assert.AreEqual(Application.unityVersion, report.unityVersion);
        Assert.AreEqual(PlayerSettings.productName, report.projectName);
        Assert.AreEqual(RemoveSuffix(Application.dataPath, "/Assets"), report.projectUrl);
    }

    [Test]
    public void TestProjectSetupSummaryGenerateReportInvalidTaskIgnored()
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
            false);

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);

        OVRConfigurationTaskUpdaterSummary taskSummary = null;
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Summary,
            onCompleted: processor => taskSummary = (processor as OVRConfigurationTaskUpdater)?.Summary);

        _generatedReportFile = taskSummary.GenerateReport("./");

        string jsonText = System.IO.File.ReadAllText(_generatedReportFile);
        Assert.That(!string.IsNullOrEmpty(jsonText));

        OVRProjectSetupJsonReport report = JsonUtility.FromJson<OVRProjectSetupJsonReport>(jsonText);

        Assert.AreEqual(0, report.tasksStatus.Count);
    }

    [Test]
    public void TestProjectSetupSummaryGenerateReportDoneTask()
    {
        var task = new OVRConfigurationTask(
            OVRProjectSetup.TaskGroup.Compatibility,
            BuildTargetGroup.Android,
            buildTargetGroup => true,
            buildTargetGroup => { },
            OVRProjectSetup.TaskLevel.Required,
            TaskMessage,
            null,
            null,
            true);

        OVRProjectSetup.AddTask(task);

        var target = BuildTargetGroup.Android;
        task.SetIgnored(target, false);

        OVRConfigurationTaskUpdaterSummary taskSummary = null;
        OVRProjectSetup.UpdateTasks(target, logMessages: OVRProjectSetup.LogMessages.Summary,
            onCompleted: processor => taskSummary = (processor as OVRConfigurationTaskUpdater)?.Summary);

        _generatedReportFile = taskSummary.GenerateReport("./");

        string jsonText = System.IO.File.ReadAllText(_generatedReportFile);
        Assert.That(!string.IsNullOrEmpty(jsonText));

        OVRProjectSetupJsonReport report = JsonUtility.FromJson<OVRProjectSetupJsonReport>(jsonText);

        Assert.AreEqual(1, report.tasksStatus.Count);
        Assert.AreEqual(true, report.tasksStatus[0].isDone);
        Assert.AreEqual(OVRProjectSetup.TaskGroup.Compatibility.ToString(), report.tasksStatus[0].group);
        Assert.AreEqual(TaskMessage, report.tasksStatus[0].message);
        Assert.AreEqual(OVRProjectSetup.TaskLevel.Required.ToString(), report.tasksStatus[0].level);
        Assert.AreEqual(task.Uid.ToString(), report.tasksStatus[0].uid);
    }

    private static string RemoveSuffix(string source, string suffix)
    {
        return source.EndsWith(suffix) ? source.Remove(source.LastIndexOf(suffix, StringComparison.Ordinal)) : source;
    }
}

#endif // OVRPLUGIN_TESTING
