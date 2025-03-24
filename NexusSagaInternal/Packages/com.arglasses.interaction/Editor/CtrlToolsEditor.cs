// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace ARGlasses.Interaction
{
    [CustomEditor(typeof(CtrlProcess))]
    [CanEditMultipleObjects]
    public class CtrlToolsEditor : Editor
    {
        private const string Src2ToolTip = "Path to the src2 repo (should end with 'src2')";
        private const string Src2BranchToolTip = "Branch in src2";
        private const string Src2YamlToolTip = "Path to YAML file in src2";

        private readonly GUIContent _src2PathGuiContent = new GUIContent("src2 Path", null, Src2ToolTip);
        private readonly GUIContent _src2BranchGuiContent = new GUIContent("Src2 Branch", null, Src2BranchToolTip);
        private readonly GUIContent _src2YamlGuiContent = new GUIContent("YAML", null, Src2YamlToolTip);

        private CtrlProcess Target => target as CtrlProcess;

        private string YamlPath => Path.GetFullPath($"{CtrlProcess.Src2Path}/{Target.Src2Yaml}");

        public override void OnInspectorGUI() // async ?
        {
            serializedObject.Update();

            EditorGUILayout.LabelField("Run the CTRL-R!");
            CtrlProcess.Src2Path = EditorGUILayout.TextField(_src2PathGuiContent, CtrlProcess.Src2Path);
            var src2Branch = EditorGUILayout.TextField(_src2BranchGuiContent, Target.Src2Branch);
            var src2Yaml = EditorGUILayout.TextField(_src2YamlGuiContent, Target.Src2Yaml);

            if (src2Branch != Target.Src2Branch || src2Yaml != Target.Src2Yaml)
            {
                Target.Src2Branch = src2Branch;
                Target.Src2Yaml = src2Yaml;
                EditorUtility.SetDirty(Target);
                serializedObject.ApplyModifiedProperties();
            }

            if (GUILayout.Button($"Run {Path.GetFileName(Target.Src2Yaml)}")) RunCtrlr();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("YAML file")) OpenYamlFile();
            if (GUILayout.Button("YAML folder")) OpenYamlFolder();
            if (GUILayout.Button("src2 folder")) EditorApplication.delayCall += () => Process.Start(CtrlProcess.Src2Path);
            GUILayout.EndHorizontal();

            if (GUILayout.Button("Open CTRL Viz")) OpenCtrlViz();
            if (GUILayout.Button("Run src2 `make install`")) RunMakeInstall();
        }


        private void RunMakeInstall()
        {
            Debug.LogWarning("TODO: Add a check for correct src2 branch");
            var lines = new List<string> { $"CALL conda activate ctrldev", $"cd {CtrlProcess.Src2Path}", $"CALL make install", };
            ExternalProcess.RunCmd(lines);
        }

        private void RunCtrlr()
        {
            Debug.LogWarning("TODO: Add a check for correct src2 branch");
            Debug.LogWarning("TODO: Add a check that `make install` has been run");

            var args = "--api-address 0.0.0.0";
            var lines = new List<string> { $"CALL conda activate ctrldev", $"CALL ctrl-r {args} -c {YamlPath}", };
            ExternalProcess.RunCmd(lines);
        }

        private const string ctrlVizURL = "https://ctrl-viz.preview.fb-ctrl.com/#screen=Line+Plot";

        public void OpenYamlFolder()
        {
            var directoryInfo = Directory.GetParent(YamlPath);
            if (directoryInfo == null) return;
            var directoryInfoFullName = directoryInfo.FullName;
            if (directoryInfo.Exists) EditorApplication.delayCall += () => { Process.Start(directoryInfoFullName); };
        }

        public void OpenYamlFile()
        {
            // var line = 0;
            var yamlPath = YamlPath;
            if (File.Exists(yamlPath))
            {
                Debug.Log($"Opening file {yamlPath}");
                Process.Start(yamlPath);
                // CodeEditor.CurrentEditor.OpenProject(yamlPath, line);
                return;
            }

            Debug.LogError($"YAML not found: {yamlPath}");
        }

        private static void OpenCtrlViz()
        {
            Application.OpenURL(ctrlVizURL);
        }

    }
}
