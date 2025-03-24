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

#if OVR_INTERNAL_CODE
// this should be kept internal as its only meant for internal usage

using UnityEditor;
using UnityEngine;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor.Compilation;
using Debug = UnityEngine.Debug;

namespace Meta.XR.BuildingBlocks.Editor
{
    public class BlockCreationWindow : EditorWindow
    {
        private string _blockName = "NewBlock";
        private string _blockDescription;
        private bool _shouldCreateNewBlockDataClass;

        private const string BlockNameKey = "BlockCreationWindow_BlockNameKey";
        private const string BlockFolderPathKey = "BlockCreationWindow_BlockFolderPathKey";
        private const string BlockDescriptionKey = "BlockCreationWindow_BlockDescriptionKey";
        private const string InternalCodeStartString = "#if OVR_INTERNAL_CODE";
        private const string InternalCodeEndString = "#endif // OVR_INTERNAL_CODE";
        private static string GetClassName(string blockName) => $"{blockName}BlockData";

        [MenuItem("Assets/Oculus/Create Building Block", false, 0)]
        private static void ShowWindow()
        {
            if (Selection.activeObject == null || Selection.activeObject is not DefaultAsset)
            {
                Debug.LogError("Please select a folder in the Project window first.");
                return;
            }

            var window = GetWindow<BlockCreationWindow>($"Create {Utils.BlockPublicName}");
            window.maxSize = new Vector2(350, 85);
            window.minSize = window.maxSize;
            window.Show();
        }

        private void OnGUI()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Name:", GUILayout.Width(80));
            _blockName = EditorGUILayout.TextField(_blockName);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Description:", GUILayout.Width(80));
            _blockDescription = EditorGUILayout.TextField(_blockDescription);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            _shouldCreateNewBlockDataClass =
                EditorGUILayout.Toggle("New Block Data Class:", _shouldCreateNewBlockDataClass);
            GUILayout.EndHorizontal();

            if (GUILayout.Button("Create Block"))
            {
                CreateBlock(_shouldCreateNewBlockDataClass);
            }
        }

        private void CreateBlock(bool newBlockDataClass)
        {
            var folderPath = AssetDatabase.GetAssetPath(Selection.activeObject);
            var blockFolderPath = Path.Combine(folderPath, _blockName);

            CreateFolder(blockFolderPath);
            CreateFolder(Path.Combine(blockFolderPath, "Shaders"));
            CreateFolder(Path.Combine(blockFolderPath, "Materials"));
            CreateFolder(Path.Combine(blockFolderPath, "Textures"));
            CreateFolder(Path.Combine(blockFolderPath, "Scripts"));

            var prefabsPath = Path.Combine(blockFolderPath, "Prefabs");
            CreateFolder(prefabsPath);

            if (newBlockDataClass)
            {
                WriteFileData(GetBlockDataClassFileData(blockFolderPath, isInternal: blockFolderPath.Contains("/OculusInternal")));

                EditorPrefs.SetString(BlockNameKey, _blockName);
                EditorPrefs.SetString(BlockDescriptionKey, _blockDescription);
                EditorPrefs.SetString(BlockFolderPathKey, blockFolderPath);

                AssetDatabase.Refresh();
                CompilationPipeline.RequestScriptCompilation();
            }
            else
            {
                CreatePrefabAndBlockData(blockFolderPath, _blockName, _blockDescription, nameof(BlockData));
            }

            Close();
        }

        [UnityEditor.Callbacks.DidReloadScripts]
        private static void OnScriptsReloaded()
        {
            if (!EditorPrefs.HasKey(BlockNameKey))
            {
                return;
            }

            var blockName = EditorPrefs.GetString(BlockNameKey);
            var blockDescription = EditorPrefs.GetString(BlockDescriptionKey);
            var blockFolderPath = EditorPrefs.GetString(BlockFolderPathKey);
            EditorPrefs.DeleteKey(BlockNameKey);
            EditorPrefs.DeleteKey(BlockDescriptionKey);
            EditorPrefs.DeleteKey(BlockFolderPathKey);

            CreatePrefabAndBlockData(blockFolderPath, blockName, blockDescription, GetClassName(blockName));
        }

        private static void CreatePrefabAndBlockData(string blockFolderPath, string blockName, string blockDescription,
            string className)
        {
            var prefabPath = Path.Combine(Path.Combine(blockFolderPath, "Prefabs"), $"{blockName}.prefab");
            CreatePrefab(blockName, prefabPath);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            EditorApplication.delayCall += () =>
            {
                var (blockData, path) =
                    CreateBlockData(className, blockName, blockDescription, prefabPath, blockFolderPath);

                WriteFileData(GetTestClassFileData(blockName, blockData.Internal), GetBlockDataIdsFileData(blockData));

                CompilationPipeline.RequestScriptCompilation();
                AssetDatabase.SaveAssets();
                AssetDatabase.Refresh();

                Selection.activeObject = AssetDatabase.LoadMainAssetAtPath(path);
            };
        }

        private static (BlockData, string) CreateBlockData(string className, string blockName, string blockDescription,
            string prefabPath, string blockFolderPath)
        {
            var blockData = (BlockData)CreateInstance(className);
            blockData.blockName = blockName;
            blockData.description = blockDescription;
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
            blockData.prefab = prefab;
            var path = Path.Combine(blockFolderPath, $"{blockName}.asset");
            AssetDatabase.CreateAsset(blockData, path);
            blockData.OnEnable();
            blockData.OnValidate();
            return (blockData, path);
        }

        private static void CreatePrefab(string blockName, string prefabPath)
        {
            var emptyGameObject = new GameObject(blockName);
            PrefabUtility.SaveAsPrefabAsset(emptyGameObject, prefabPath);
            DestroyImmediate(emptyGameObject);
        }

        #region File Generation

        private static void CreateFolder(string path)
        {
            if (!AssetDatabase.IsValidFolder(path))
            {
                AssetDatabase.CreateFolder(Path.GetDirectoryName(path), Path.GetFileName(path));
            }
        }

        private readonly struct FileData
        {
            public readonly string FilePath;
            public readonly string Content;
            public FileData(string filePath, string content)
            {
                FilePath = filePath;
                Content = content;
            }
        }

        private static void WriteFileData(params FileData[] fileDataArray)
        {
            foreach (var fileData in fileDataArray)
            {
                File.WriteAllText(fileData.FilePath, fileData.Content);
            }
        }

        private static FileData GetBlockDataIdsFileData(BlockBaseData blockData)
        {
            const string path = "Assets/Oculus/VR/Editor/BuildingBlocks/BlockDataIds.cs";

            return File.ReadAllLines(path)
                .SelectMany(ParseLine)
                .Aggregate(new StringBuilder(), (acc, line) => acc.AppendLine(line))
                .Let(fileContent => new FileData(path, fileContent.ToString()));

            bool ShouldAddNewLine(string line) =>
                (blockData.Internal, line.Trim()) switch
                {
                    (false, InternalCodeStartString) => true,
                    (true, InternalCodeEndString) => true,
                    _ => false,
                };

            string[] ParseLine(string line) =>
                ShouldAddNewLine(line)
                    ? new[] { $"        public const string {blockData.BlockName} = \"{blockData.Id}\";\n", line }
                    : new[] { line };
        }

        private FileData GetBlockDataClassFileData(string blockFolderPath, bool isInternal)
        {
            var scriptsPath = Path.Combine(blockFolderPath, "Scripts");
            var filePath = Path.Combine(scriptsPath, $"{GetClassName(_blockName)}.cs");

            var sourceCode = $@"/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the ""License"");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

{(isInternal ? InternalCodeStartString + "\n" : "")}
using System.Collections.Generic;
using UnityEngine;

namespace Meta.XR.BuildingBlocks.Editor
{{
    public class {GetClassName(_blockName)} : BlockData
    {{
        protected override List<GameObject> InstallRoutine(GameObject selectedGameObject)
        {{
            return base.InstallRoutine(selectedGameObject);
        }}
    }}
}}
{(isInternal ? InternalCodeEndString + "\n" : "")}
";

            return new FileData (filePath, sourceCode);
        }

        private static FileData GetTestClassFileData(string blockName, bool isInternal)
        {
            const string testsPath = "Assets/Oculus/VR/Scripts/OculusInternal/Tests/PlayMode/BuildingBlocks/TestCases";
            var className = $"{blockName}BlocksTest";
            var filePath = Path.Combine(testsPath, $"{className}.cs");

            var sourceCode = $@"/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the ""License"");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if OVRPLUGIN_TESTING
{(isInternal ? InternalCodeStartString + "\n" : "")}
using System.Collections;
using System.Collections.Generic;
using Meta.XR.BuildingBlocks.Editor;

public class {className} : BuildingBlocksTest
{{
    public override string BlockId => BlockDataIds.{blockName};

    public override IEnumerator Setup(TestContext testContext)
    {{
        yield return null;
    }}

    public override IEnumerator TearDown(TestContext testContext)
    {{
        yield return null;
    }}

    public override List<IEnumerator> TestCaseList => new();
}}
{(isInternal ? InternalCodeEndString + "\n" : "")}
#endif //OVRPLUGIN_TESTING
";

            return new FileData (filePath, sourceCode);
        }

        #endregion
    }
}

#endif // OVR_INTERNAL_CODE
