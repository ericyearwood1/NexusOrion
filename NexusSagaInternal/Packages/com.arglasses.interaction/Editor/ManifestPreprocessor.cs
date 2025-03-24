// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using Unity.CodeEditor;
using UnityEngine;
using static ARGlasses.Interaction.ExternalProcess;

namespace ARGlasses.Interaction
{
    [InitializeOnLoad]
    class ManifestPreprocessor : AssetPostprocessor
    {
        public const string PyFileName = "gitmanifest.py";
        private static readonly string DidRefreshThisSessionKey = $"{nameof(ManifestPreprocessor)}";
        private static System.Diagnostics.Process _pyProcess;

        static ManifestPreprocessor()
        {
            if (!DidRefreshThisSession) Refresh();
        }

        public static bool HasGitEntryInManifest => Manifest.ContainsKey("git");

        private static bool DidRefreshThisSession
        {
            get => SessionState.GetBool(DidRefreshThisSessionKey, false);
            set => SessionState.SetBool(DidRefreshThisSessionKey, value);
        }

        [MenuItem("Window/GitManifest/Edit Packages", priority = 1500)]
        public static void EditGitPackages()
        {
            // todo Parse manifest.json for the correct starting line of the git section
            var line = 0;
            CodeEditor.CurrentEditor.OpenProject(ManifestJson, line);
        }

        [MenuItem("Window/GitManifest/Run Manifest Update", priority = 1500)]
        public static void Refresh()
        {
            if (!HasGitEntryInManifest) return;
            
            Debug.Log($"Attempting to refresh manifest.json from {nameof(ManifestPreprocessor)}...");
            DidRefreshThisSession = true;

            // todo escape this if manifest.json is not dirty
            var sideloaderFiles = FindFilesByName(PyFileName).ToList();
            if (sideloaderFiles.Count != 1) Debug.LogError("Found multiple instances of sideloader.py");
            var pyFileAbsolute = CombineFull(ProjectRoot, sideloaderFiles[0]);
            Stop(_pyProcess);
            _pyProcess = RunPyFile(pyFileAbsolute, spawnTerminal: true, args: new List<string> { ManifestJson });
        }

        protected void OnPreprocessAsset()
        {
            // if (EditorApplication.timeSinceStartup - LastRefreshTime < 5) return;
            // if (assetPath.EndsWith("sideloader.json", StringComparison.OrdinalIgnoreCase)) Refresh();
        }
    }
}
