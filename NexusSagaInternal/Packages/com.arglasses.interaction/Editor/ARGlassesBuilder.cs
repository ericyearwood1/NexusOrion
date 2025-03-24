using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using JetBrains.Annotations;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using static ARGlasses.Interaction.ExtensionsUnityEditor;
using Debug = UnityEngine.Debug;

namespace ARGlasses.Interaction
{
    [CustomEditor(typeof(ARGlassesBuilderControl), true)]
    [CanEditMultipleObjects]
    public class ARGlassesBuilderControlEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            ARGlassesBuilderControl control = target as ARGlassesBuilderControl;
            ARGlassesBuilder builder = control.builder as ARGlassesBuilder;
            if (builder) ARGlassesBuilder.ARGlassesBuilderEditor.RenderBuilderOptions(builder);
        }
    }

    [CreateAssetMenu(fileName = "ARGlassesBuilder", menuName = "ARGlasses/Builder", order = 1)]
    public class ARGlassesBuilder : ScriptableObject
    {
        [CustomEditor(typeof(ARGlassesBuilder), true)]
        [CanEditMultipleObjects]
        public class ARGlassesBuilderEditor : Editor
        {
            private static readonly string DeveloperFlagKey = $"{nameof(ARGlassesBuilderControlEditor)}.{nameof(DeveloperFlagKey)}";

            public static bool DevelopmentMode
            {
                get => EditorGetBool(DeveloperFlagKey, true);
                set => EditorSetBool(DeveloperFlagKey, value);
            }

            private static readonly string RunFlagKey = $"{nameof(ARGlassesBuilderControlEditor)}.{nameof(RunFlagKey)}";

            public static bool RunMode
            {
                get => EditorGetBool(RunFlagKey, false);
                set => EditorSetBool(RunFlagKey, value);
            }

            private static readonly string PushGetAppKey = $"{nameof(ARGlassesBuilderControlEditor)}.{nameof(PushGetAppKey)}";

            public static bool PushGetApp
            {
                get => EditorGetBool(PushGetAppKey, false);
                set => EditorSetBool(PushGetAppKey, value);
            }

            private static void Header(string label)
            {
                var guiStyle = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 18,
                    normal = { textColor = Color.white },
                    alignment = TextAnchor.UpperLeft,
                    fontStyle = FontStyle.Bold
                };
                GUILayout.Label(label, guiStyle);
            }

            public override void OnInspectorGUI()
            {
                DrawDefaultInspector();
                RenderBuilderOptions(target as ARGlassesBuilder);
            }

            public static void RenderBuilderOptions(ARGlassesBuilder builder)
            {
                EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
                Header("Build");

                RunMode = GUILayout.Toggle(RunMode, new GUIContent($"Auto Run on Quest", ""));
                DevelopmentMode = GUILayout.Toggle(DevelopmentMode, new GUIContent($"Development Build", "Initiate a build with developer mode enabled (can't deploy to GetApp)"));

                var pushGetAppAllowed = !DevelopmentMode;

                GUI.enabled = pushGetAppAllowed;
                PushGetApp = GUILayout.Toggle(PushGetApp, new GUIContent($"Publish to GetApp {(DevelopmentMode ? "(DeveloperMode must be disabled)" : "")}", ""));
                PushGetApp &= pushGetAppAllowed;
                GUI.enabled = true;

                var buildButtonLabel = $"Build";
                if (RunMode) buildButtonLabel += " and Run";
                if (DevelopmentMode) buildButtonLabel += " Developement";
                if (PushGetApp) buildButtonLabel += " then Push to GetApp";

                if (GUILayout.Button(new GUIContent(buildButtonLabel, "Initiate a build with developer mode enabled (can't deploy to GetApp)")))
                {
                    builder.Build(devMode: DevelopmentMode, runPlayer: RunMode);
                    if (PushGetApp) DeployToGetApp(builder);
                    // ShowProfiler();
                    // if(RunMode) InstallAndRunOnQuest(builder);
                }

                EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
                Header("Previous Build");

                if (GUILayout.Button("Open Builds folder")) builder.OpenBuildsFolder();
                if (GUILayout.Button("Install and Run")) InstallAndRunOnQuest(builder);
                if (GUILayout.Button("Deploy to GetApp")) DeployToGetApp(builder);
                if (GUILayout.Button("Show Profiler")) ShowProfiler();

                EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
                Header("Resources");
                if (GUILayout.Button("Open Admin page")) Application.OpenURL(builder.AdminUrl);
                if (GUILayout.Button("Open GetApp page")) Application.OpenURL(builder.GetAppUrl);
                if (GUILayout.Button("Open App Creation docs")) Application.OpenURL(builder.DocsUrl);
            }

            private static void ShowProfiler()
            {
                EditorWindow.GetWindow<ProfilerWindow>().Show();
            }

            private static void DeployToGetApp(ARGlassesBuilder builder)
            {
                var protoPush = ExternalProcess.ProjectRelativePath("../../scripts/ovr-proto-push.exe");
                var apkLocation = builder.LocationPathNameFull;
                var description = $"v{PlayerSettings.bundleVersion}, {PrettyDateTime}";
                var cmd = $"\"{protoPush}\" \"{builder.GetAppName}\" \"{apkLocation}\" --description \"{description}\"";
                if (!string.IsNullOrEmpty(builder.GetAppToken)) cmd += $" --token {builder.GetAppToken}";
                RunCmd(cmd);
            }

            private static void InstallAndRunOnQuest(ARGlassesBuilder builder)
            {
                RunCmds(new List<string>
                    {
                        $"maui adb install -r -d \"{builder.LocationPathNameFull}\"",
                        $"maui adb shell am start -n {builder.PackageName}/com.unity3d.player.UnityPlayerActivity"
                    }
                );
            }

            private static void RunCmd(string cmd)
            {
                RunCmds(new List<string> { cmd });
            }

            private static void RunCmds(List<string> cmds, bool echo = false)
            {
                var fullCmds = new List<string>();
                foreach (var cmd in cmds)
                {
                    if (echo) fullCmds.Add($"echo {cmd}");
                    fullCmds.Add(cmd);
                }

                ExternalProcess.RunCmd(fullCmds);
                // process.WaitForExit();
            }

            private static void CallMultiple(Action<ARGlassesBuilder> builderAction)
            {
                foreach (var obj in UnityEditor.Selection.objects) builderAction(obj as ARGlassesBuilder);
            }
        }

        public static string PrettyDateTime
        {
            get
            {
                var pacificZone = TimeZoneInfo.FindSystemTimeZoneById("Pacific Standard Time");
                var pacificTime = TimeZoneInfo.ConvertTimeFromUtc(DateTime.UtcNow, pacificZone);
                return pacificTime.ToString("g").ToLower() + " PT";
            }
        }

        public string FileExtension
        {
            get
            {
                if (BuildTargetGroup == BuildTargetGroup.Android) return "apk";
                if (BuildTargetGroup == BuildTargetGroup.Standalone) return "exe";
                return string.Empty;
            }
        }

        [SerializeField] private BuildTarget _buildTarget = BuildTarget.Android;
        [SerializeField] private string _productName;
        [SerializeField] private SceneAsset[] _scenes = Array.Empty<SceneAsset>();
        [SerializeField] private string _packageName = null;
        [SerializeField] private bool _incrementVersion = true;
        [SerializeField] private string _locationPathName;
        [SerializeField] private string _getAppName;
        [SerializeField] private string _getAppId;
        [SerializeField] private string _getAppToken;
        [SerializeField, ReadOnly] private BuildReport _lastBuildReport;

        public bool IncrementVersion => _incrementVersion;
        public string ProductName => !string.IsNullOrEmpty(_productName) ? _productName : PlayerSettings.productName;
        public string PackageName => !string.IsNullOrEmpty(_packageName) ? _packageName : PlayerSettings.GetApplicationIdentifier(BuildTargetGroup);
        public BuildTarget BuildTarget => _buildTarget;
        public BuildTargetGroup BuildTargetGroup => BuildPipeline.GetBuildTargetGroup(_buildTarget);
        public string LocationPathName => !string.IsNullOrEmpty(_locationPathName) ? _locationPathName : $"Builds/{BuildTarget.ToStringName()}";
        public string LocationPathNameFull => Path.Combine(ExternalProcess.ProjectRoot, LocationPathName);

        public string GetAppName => _getAppName;
        public string GetAppId => _getAppId;
        public string GetAppToken => _getAppToken;

        public SceneAsset[] Scenes => _scenes;
        public string AdminUrl => $"https://developer.oculus.com/manage/applications/{GetAppId}";
        public string GetAppUrl => $"https://www.internalfb.com/intern/oculus/application/{GetAppId}/get/";
        public string DocsUrl => $"https://www.internalfb.com/intern/wiki/RL/Design/Prototyping/Prototype_Distribution/Using_Meta_Quest_Developer_Hub/";

        public void OpenBuildsFolder() => EditorUtility.RevealInFinder(LocationPathName);

        private string[] SceneAssetPaths => Scenes.Select(AssetDatabase.GetAssetPath).ToArray();

        public void SwitchToPlatform()
        {
            // PlayerSettings.SetScriptingBackend(BuildTargetGroup.Standalone, ScriptingImplementation.Mono2x);
            // PlayerSettings.SetScriptingBackend(BuildTargetGroup.WSA, ScriptingImplementation.IL2CPP);
            Debug.Log($"Switching to {BuildTarget}");
            EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup, BuildTarget);
            EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo();

            if (_scenes == null || _scenes.Length <= 0) return;
            EditorBuildSettings.scenes = SceneAssetPaths.Select(ap => new EditorBuildSettingsScene(ap, true)).ToArray();
            EditorSceneManager.OpenScene(SceneAssetPaths[0]);
        }

        [UsedImplicitly]
        public static void BuildFromArgs()
        {
            try
            {
                var args = CommandLineUtil.GetCommandLineArguments();
                args.TryGetValue("-builderPath", out var builderPath);
                args.TryGetValue("-outDir", out var outDir);
                args.TryGetValue("-productName", out var productName);
                args.TryGetValue("-packageName", out var packageName);

                var builder = AssetDatabase.LoadAssetAtPath<ARGlassesBuilder>(builderPath);
                // var asset = AssetDatabase.GetAssetPath(path);

                builder.Build();
            }
            catch (Exception e)
            {
                Debug.LogError("ARGlasses.Interaction.CommandLineBuilder failed");
                Debug.LogError(e);
            }
            finally
            {
                EditorApplication.Exit(0);
            }
        }

        [SerializeField] private BuildOptions _devModeOptions = BuildOptions.Development |
                                                                BuildOptions.AllowDebugging |
                                                                BuildOptions.ConnectWithProfiler; //BuildOptions.ConnectToHost

        public void Build(bool devMode = false, bool runPlayer = false)
        {
            EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup, BuildTarget);


            TryIncrementVersion();

            PlayerSettings.productName = ProductName;
            PlayerSettings.SplashScreen.show = false;
            PlayerSettings.SetApplicationIdentifier(BuildTargetGroup, PackageName);
            BuildOptions buildOptions = BuildOptions.None;

            if (devMode) buildOptions |= _devModeOptions;
            if (runPlayer) buildOptions |= BuildOptions.AutoRunPlayer;

            var options = new BuildPlayerOptions
            {
                target = BuildTarget,
                scenes = SceneAssetPaths,
                options = buildOptions,
                locationPathName = LocationPathName,
            };

            string log = $"Building {BuildTarget} {ProductName} {LocationPathName}\n";
            foreach (var assetPath in SceneAssetPaths) log += ", " + assetPath;
            log += "\n";
            log += buildOptions;
            Debug.Log(log);

            EditorApplication.RepaintProjectWindow();

            OpenBuildsFolder();

            var stopwatch = Stopwatch.StartNew();
            _lastBuildReport = BuildPipeline.BuildPlayer(options);
            stopwatch.Stop();

            var elapsed = stopwatch.Elapsed;
            Debug.Log($"Build time: {elapsed.Minutes:00}:{elapsed.Seconds:00}");

            if (_lastBuildReport.summary.result == BuildResult.Succeeded) EditorSound.PlaySuccess();
            else EditorSound.PlayFail();
        }

        private void TryIncrementVersion()
        {
            PreventProtoKitVersioningHack();
            if (!IncrementVersion) return;
            PlayerSettings.Android.bundleVersionCode++;
            var prevVersionStrings = PlayerSettings.bundleVersion.Split('.');
            string[] versionStrings = { "0", "0", "0", PlayerSettings.Android.bundleVersionCode.ToString() };
            versionStrings[0] = !string.IsNullOrEmpty(prevVersionStrings[0]) ? prevVersionStrings[0] : "0";
            if (prevVersionStrings.Length > 1) versionStrings[1] = prevVersionStrings[1];
            if (prevVersionStrings.Length > 2) versionStrings[2] = prevVersionStrings[2];

            PlayerSettings.bundleVersion = string.Join(".", versionStrings);
            Debug.Log($"Updated PlayerSettings.bundleVersion to {PlayerSettings.bundleVersion}");
        }

        private void PreventProtoKitVersioningHack()
        {
            const string INCREMENT_VERSION_ON_BUILD_KEY = "Protokit_Core_IncrementVersionOnBuild";
            PlayerPrefs.SetInt(INCREMENT_VERSION_ON_BUILD_KEY, 0);
        }
    }
}
