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

using System.Collections.Generic;
using UnityEditor;
using System;
using System.Linq;
using Meta.XR.Editor.StatusMenu;
using Meta.XR.Editor.UserInterface;
using UnityEngine;
using Styles = Meta.XR.Editor.UserInterface.Styles;
using static Meta.XR.Editor.UserInterface.Styles.Colors;

/// <summary>
/// Core System for the OVRProjectSetup Tool
/// </summary>
/// <remarks>
/// This static class manages <see cref="OVRConfigurationTask"/> that can be added at any point.
/// Use the AddTask method to add and register <see cref="OVRConfigurationTask"/>.
/// </remarks>
public static class OVRProjectSetup
{
    public enum TaskLevel
    {
        Optional = 0,
        Recommended = 1,
        Required = 2
    }

    public enum TaskGroup
    {
        All = 0,
        Compatibility = 1,
        Rendering = 2,
        Quality = 3,
        Physics = 4,
        Packages = 5,
        Features = 6,
        Miscellaneous = 7
    }

    private static readonly OVRConfigurationTaskRegistry _principalRegistry;

    internal static OVRConfigurationTaskRegistry Registry { get; private set; }
    internal static OVRConfigurationTaskProcessorQueue ProcessorQueue { get; }

    internal const string KeyPrefix = "OVRProjectSetup";
    internal static OVRProjectSetupSettingBool Enabled;
    internal static OVRProjectSetupSettingBool RequiredThrowErrors;

    internal static readonly OVRProjectSetupSettingBool AllowLogs =
        new OVRProjectSetupProjectSettingBool("AllowLogs", false, "Log outstanding issues");

    internal static readonly OVRProjectSetupSettingBool ShowStatusIcon =
        new OVRProjectSetupProjectSettingBool("ShowStatusIcon", true, "Show Status Icon");

    internal static readonly OVRProjectSetupSettingBool ProduceReportOnBuild =
        new OVRProjectSetupProjectSettingBool("ProduceReportOnBuild", false, "Produce Report on Build");

    private static readonly HashSet<BuildTargetGroup> SupportedPlatforms = new HashSet<BuildTargetGroup>
        { BuildTargetGroup.Android, BuildTargetGroup.Standalone };

#if OVRPLUGIN_TESTING
    public static bool IsUsingPrincipalRegistry => Registry == _principalRegistry;
    public static bool IsReady => IsUsingPrincipalRegistry && !ProcessorQueue.Busy;
#endif

    internal const string PublicName = "Project Setup Tool";

    internal static readonly TextureContent StatusIcon = TextureContent.CreateContent("ovr_icon_upst.png",
        OVRProjectSetupUtils.ProjectSetupToolIcons, $"Open {PublicName}");

    private const string DocumentationUrl ="https://developer.oculus.com/documentation/unity/unity-upst-overview";

#if OVR_INTERNAL_CODE
    private const string WorkplaceUrl = "https://fb.workplace.com/groups/339438274734616/permalink/530044969007278/";
    private const string WikiUrl =
        "https://www.internalfb.com/intern/wiki/Oculus/Vision/MixedReality/Presence_Platform_Frameworks/Unity/Unity_Project_Setup_Tool";
    private const string FeedbackUrl = "https://www.fburl.com/ppt-feedback";
#endif

    internal static Item Item = new Item()
    {
        Name = PublicName,
        Color = Styles.Colors.BrightGray,
        Icon = StatusIcon,
        InfoTextDelegate = ComputeInfoText,
        PillIcon = ComputePillIcon,
        OnClickDelegate = OnStatusMenuClick,
        Order = 0,
        HeaderIcons = new List<Item.HeaderIcon>()
        {
            new Item.HeaderIcon()
            {
                TextureContent = Styles.Contents.ConfigIcon,
                Color = LightGray,
                Action = OVRProjectSetupDrawer.ShowSettingsMenu
            },
            new Item.HeaderIcon()
            {
                TextureContent = Styles.Contents.DocumentationIcon,
                Color = LightGray,
                Action = () => Application.OpenURL(DocumentationUrl)
            },
#if OVR_INTERNAL_CODE
            new Item.HeaderIcon()
            {
                TextureContent = Styles.Contents.WikiIcon,
                Color = InternalColor,
                Action = () => Application.OpenURL(WikiUrl)
            },
            new Item.HeaderIcon()
            {
                TextureContent = Styles.Contents.WorkplaceIcon,
                Color = InternalColor,
                Action = () => Application.OpenURL(WorkplaceUrl)
            },
            new Item.HeaderIcon()
            {
                TextureContent = Styles.Contents.FeedbackIcon,
                Color = InternalColor,
                Action = () => Application.OpenURL(FeedbackUrl)
            }
#endif
        }
    };

    static OVRProjectSetup()
    {
        _principalRegistry = new OVRConfigurationTaskRegistry();
        ProcessorQueue = new OVRConfigurationTaskProcessorQueue();
        ConsoleLinkEventHandler.OnConsoleLink += OnConsoleLink;
        RestoreRegistry();

        ProcessorQueue.OnProcessorCompleted += RefreshBuildStatusMenuSubText;
        var statusItem = new Item()
        {
            Name = OVRProjectSetupUtils.ProjectSetupToolPublicName,
            Color = Utils.HexToColor("#c4c4c4"),
            Icon = TextureContent.CreateContent("ovr_icon_upst.png", OVRProjectSetupUtils.ProjectSetupToolIcons),
            InfoTextDelegate = ComputeInfoText,
            PillIcon = ComputePillIcon,
            OnClickDelegate = OnStatusMenuClick,
            Order = 0
        };
        StatusMenu.RegisterItem(statusItem);
    }

    private static string _statusMenuSubText;
    private static OVRConfigurationTaskUpdaterSummary _latestSummary;

    private static void RefreshBuildStatusMenuSubText(OVRConfigurationTaskProcessor processor)
    {
        var updater = processor as OVRConfigurationTaskUpdater;
        var summary = updater?.Summary;
        _statusMenuSubText = summary?.ComputeNoticeMessage();
        _latestSummary = summary;
    }

    private static (string, Color?) ComputeInfoText() => (_statusMenuSubText, null);

    private static (TextureContent, Color?) ComputePillIcon()
    {
        return _latestSummary?.HighestFixLevel switch
        {
            OVRProjectSetup.TaskLevel.Optional => (OVRProjectSetupDrawer.Styles.Contents.InfoIcon, InfoColor),
            OVRProjectSetup.TaskLevel.Recommended => (OVRProjectSetupDrawer.Styles.Contents.WarningIcon, WarningColor),
            OVRProjectSetup.TaskLevel.Required => (OVRProjectSetupDrawer.Styles.Contents.ErrorIcon, ErrorColor),
            _ => (null, null)
        };
    }

    private static void OnStatusMenuClick(Item.Origins origin)
    {
        OVRProjectSetupSettingsProvider.OpenSettingsWindow(origin);
    }

    internal static void SetupTemporaryRegistry()
    {
        Registry = new OVRConfigurationTaskRegistry();
        Enabled = new OVRProjectSetupConstSettingBool("Enabled", true, "Enabled");
        RequiredThrowErrors =
            new OVRProjectSetupConstSettingBool("RequiredThrowErrors", false, "Required throw errors");
        OVRProjectSetupUpdater.SetupTemporaryRegistry();
    }

    internal static void RestoreRegistry()
    {
        Registry = _principalRegistry;
        Enabled =
#if OVR_INTERNAL_CODE
            new OVRProjectSetupProjectSettingBool("Enabled", true, "Enabled");
#else
        new OVRProjectSetupConstSettingBool("Enabled", true, "Enabled");
#endif
        RequiredThrowErrors =
            new OVRProjectSetupProjectSettingBool("RequiredThrowErrors", false, "Required throw errors");
        OVRProjectSetupUpdater.RestoreRegistry();
    }

    private static void OnConsoleLink(Dictionary<string, string> infos)
    {
        if (infos.TryGetValue("href", out var href))
        {
            if (href == OVRConfigurationTask.ConsoleLinkHref)
            {
                OVRProjectSetupSettingsProvider.OpenSettingsWindow(Item.Origins.Console);
            }
        }
    }

    internal static IEnumerable<OVRConfigurationTask> GetTasks(BuildTargetGroup buildTargetGroup, bool refresh)
    {
        return Registry.GetTasks(buildTargetGroup, refresh);
    }

    /// <summary>
    /// Add an <see cref="OVRConfigurationTask"/> to the Setup Tool.
    /// </summary>
    /// <remarks>
    /// This methods adds and registers an already created <see cref="OVRConfigurationTask"/> to the SetupTool.
    /// We recommend the use of the other AddTask method with all the required parameters to create the task.
    /// </remarks>
    /// <param name="task">The task that will get registered to the Setup Tool.</param>
    /// <exception cref="ArgumentException">Possible causes :
    /// - a task with the same unique ID already has been registered (conflict in hash generated from description message).</exception>
    internal static void AddTask(OVRConfigurationTask task)
    {
        Registry.AddTask(task);
    }

    /// <summary>
    /// Add an <see cref="OVRConfigurationTask"/> to the Setup Tool.
    /// </summary>
    /// <remarks>
    /// This methods creates, adds and registers an <see cref="OVRConfigurationTask"/> to the SetupTool.
    /// Please note that the Message or ConditionalMessage parameters have to be unique since they are being hashed to generate a Unique ID for the task.
    /// Those tasks, once added, are not meant to be removed from the Setup Tool, and will get checked at some key points.
    /// This method is the one entry point for developers to add their own sanity checks, technical requirements or other recommendations.
    /// You can use the conditional parameters that accepts lambdas or delegates for more complex behaviours if needed.
    /// </remarks>
    /// <param name="group">Category that fits the task. Feel free to add more to the enum if relevant. Do not use "All".</param>
    /// <param name="isDone">Delegate that checks if the Configuration Task is validated or not.</param>
    /// <param name="platform">Platform for which this Configuration Task applies. Use "Unknown" for any.</param>
    /// <param name="fix">Delegate that validates the Configuration Task.</param>
    /// <param name="level">Severity (or behaviour) of the Configuration Task.</param>
    /// <param name="conditionalLevel">Use this delegate for more control or complex behaviours over the level parameter.</param>
    /// <param name="message">Description of the Configuration Task.</param>
    /// <param name="conditionalMessage">Use this delegate for more control or complex behaviours over the message parameter.</param>
    /// <param name="fixMessage">Description of the actual fix for the Task.</param>
    /// <param name="conditionalFixMessage">Use this delegate for more control or complex behaviours over the fixMessage parameter.</param>
    /// <param name="url">Url to more information about the Configuration Task.</param>
    /// <param name="conditionalUrl">Use this delegate for more control or complex behaviours over the url parameter.</param>
    /// <param name="validity">Checks if the task is valid. If not, it will be ignored by the Setup Tool.</param>
    /// <param name="conditionalValidity">Use this delegate for more control or complex behaviours over the validity parameter.</param>
    /// <exception cref="ArgumentNullException">Possible causes :
    /// - If either message or conditionalMessage do not provide a valid non null string
    /// - isDone is null
    /// - fix is null</exception>
    /// <exception cref="ArgumentException">Possible causes :
    /// - group is set to "All". This category is not meant to be used to describe a task.
    /// - a task with the same unique ID already has been registered (conflict in hash generated from description message).</exception>
    public static void AddTask(
        OVRProjectSetup.TaskGroup group,
        Func<BuildTargetGroup, bool> isDone,
        BuildTargetGroup platform = BuildTargetGroup.Unknown,
        Action<BuildTargetGroup> fix = null,
        OVRProjectSetup.TaskLevel level = OVRProjectSetup.TaskLevel.Recommended,
        Func<BuildTargetGroup, OVRProjectSetup.TaskLevel> conditionalLevel = null,
        string message = null,
        Func<BuildTargetGroup, string> conditionalMessage = null,
        string fixMessage = null,
        Func<BuildTargetGroup, string> conditionalFixMessage = null,
        string url = null,
        Func<BuildTargetGroup, string> conditionalUrl = null,
        bool validity = true,
        Func<BuildTargetGroup, bool> conditionalValidity = null
    )
    {
        var optionalLevel =
            OptionalLambdaType<BuildTargetGroup, OVRProjectSetup.TaskLevel>.Create(level, conditionalLevel, true);
        var optionalMessage = OptionalLambdaType<BuildTargetGroup, string>.Create(message, conditionalMessage, true);
        var optionalFixMessage =
            OptionalLambdaType<BuildTargetGroup, string>.Create(fixMessage, conditionalFixMessage, true);
        var optionalUrl = OptionalLambdaType<BuildTargetGroup, string>.Create(url, conditionalUrl, true);
        var optionalValidity = OptionalLambdaType<BuildTargetGroup, bool>.Create(validity, conditionalValidity, true);
        AddTask(new OVRConfigurationTask(group, platform, isDone, fix, optionalLevel, optionalMessage,
            optionalFixMessage, optionalUrl, optionalValidity));
    }

    internal static bool IsPlatformSupported(BuildTargetGroup buildTargetGroup)
    {
        return SupportedPlatforms.Contains(buildTargetGroup);
    }

    internal enum LogMessages
    {
        Disabled = 0,
        Summary = 1,
        Changed = 2,
        All = 3,
    }

    private const int LoopExitCount = 4;

    internal static void FixTasks(
        BuildTargetGroup buildTargetGroup,
        Func<IEnumerable<OVRConfigurationTask>, List<OVRConfigurationTask>> filter = null,
        LogMessages logMessages = LogMessages.Disabled,
        bool blocking = true,
        Action<OVRConfigurationTaskProcessor> onCompleted = null)
    {
        var fixer = new OVRConfigurationTaskFixer(Registry, buildTargetGroup, filter, logMessages, blocking,
            onCompleted);
        ProcessorQueue.Request(fixer);
    }

    internal static void FixTask(
        BuildTargetGroup buildTargetGroup,
        OVRConfigurationTask task,
        LogMessages logMessages = LogMessages.Disabled,
        bool blocking = true,
        Action<OVRConfigurationTaskProcessor> onCompleted = null
    )
    {
        // TODO : A bit overkill for just one task
        var filter = (Func<IEnumerable<OVRConfigurationTask>, List<OVRConfigurationTask>>)(tasks =>
            tasks.Where(otherTask => otherTask == task).ToList());
        var fixer = new OVRConfigurationTaskFixer(Registry, buildTargetGroup, filter, logMessages, blocking,
            onCompleted);
        ProcessorQueue.Request(fixer);
    }

    internal static void UpdateTasks(
        BuildTargetGroup buildTargetGroup,
        Func<IEnumerable<OVRConfigurationTask>, List<OVRConfigurationTask>> filter = null,
        LogMessages logMessages = LogMessages.Disabled,
        bool blocking = true,
        Action<OVRConfigurationTaskProcessor> onCompleted = null)
    {
        var updater =
            new OVRConfigurationTaskUpdater(Registry, buildTargetGroup, filter, logMessages, blocking, onCompleted);
        ProcessorQueue.Request(updater);
    }
}
