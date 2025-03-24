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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using Meta.XR.Editor.Callbacks;
using Meta.XR.Editor.StatusMenu;
using Meta.XR.Editor.Tags;
using Meta.XR.Editor.UserInterface;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using static Meta.XR.Editor.UserInterface.Styles.Colors;
using static Meta.XR.Editor.UserInterface.Styles.Contents;
using Object = UnityEngine.Object;

#if OVR_INTERNAL_CODE
using System.IO;
using System.Reflection;
using JetBrains.Annotations;
#endif // OVR_INTERNAL_CODE

namespace Meta.XR.BuildingBlocks.Editor
{
    [InitializeOnLoad]
    internal static class Utils
    {
        internal const string BlocksPublicName = "Building Blocks";
        internal const string BlockPublicName = "Building Block";
        internal const string BlockPublicTag = "[BuildingBlock]";

        internal static readonly TextureContent.Category BuildingBlocksIcons = new("BuildingBlocks/Icons");
        internal static readonly TextureContent.Category BuildingBlocksThumbnails = new("BuildingBlocks/Thumbnails");
        internal static readonly TextureContent.Category BuildingBlocksAnimations = new("BuildingBlocks/Animations");

        internal static readonly TextureContent StatusIcon = TextureContent.CreateContent("ovr_icon_bbw.png",
            Utils.BuildingBlocksIcons, $"Open {BlocksPublicName}");

        internal static readonly TextureContent GotoIcon = TextureContent.CreateContent("ovr_icon_link.png",
            Utils.BuildingBlocksIcons, "Select Block");

        internal static readonly TextureContent AddIcon = TextureContent.CreateContent("ovr_icon_addblock.png",
            Utils.BuildingBlocksIcons, "Add Block to current scene");

        private const string ExperimentalTagName = "Experimental";

        internal static readonly TextureContent ExperimentalIcon =
            TextureContent.CreateContent("ovr_icon_experimental.png", Utils.BuildingBlocksIcons,
                ExperimentalTagName);

        internal static Tag ExperimentalTag = new(ExperimentalTagName)
        {
            Behavior =
            {
                Color = ExperimentalColor,
                Icon = ExperimentalIcon,
                Order = 100,
                ShowOverlay = true,
                ToggleableVisibility = true,
            }
        };

        private const string PrototypingTagName = "Prototyping";

        internal static readonly TextureContent PrototypingIcon =
            TextureContent.CreateContent("ovr_icon_prototype.png", Utils.BuildingBlocksIcons,
                PrototypingTagName);

        internal static Tag PrototypingTag = new(PrototypingTagName)
        {
            Behavior =
            {
                Color = ExperimentalColor,
                Icon = PrototypingIcon,
                Order = 101,
                ShowOverlay = true,
                ToggleableVisibility = true,
            }
        };

        private const string DebugTagName = "Debug";

        internal static readonly TextureContent DebugIcon =
            TextureContent.CreateContent("ovr_icon_debug.png", Utils.BuildingBlocksIcons, DebugTagName);

        internal static Tag DebugTag = new(DebugTagName)
        {
            Behavior =
            {
                Color = DebugColor,
                Icon = DebugIcon,
                Order = 90,
                ShowOverlay = true,
                ToggleableVisibility = true,
            }
        };

        private const string InternalTagName = "Internal";

        internal static Tag InternalTag = new(InternalTagName)
        {
            Behavior =
            {
                Order = 200,
                Automated = true,
#if OVR_INTERNAL_CODE
                Color = InternalColor,
                Icon = TextureContent.CreateContent("ovr_icon_meta.png", Utils.BuildingBlocksIcons,
                    InternalTagName),
                Show = true,
                ShowOverlay = true,
                ToggleableVisibility = true,
#else
                Show = false,
#endif
                DefaultVisibility = false
            }
        };

        private const string HiddenTagName = "Hidden";

        internal static Tag HiddenTag = new(HiddenTagName)
        {
            Behavior =
            {
                Order = 201,
#if OVR_INTERNAL_CODE
                Color = LightGray,
                Icon = TextureContent.CreateContent("ovr_icon_hidden.png", Utils.BuildingBlocksIcons,
                    HiddenTagName),
                Show = true,
                ShowOverlay = true,
                ToggleableVisibility = true,
#else
                Show = false,
#endif
                DefaultVisibility = false,
            }
        };

        private const string DeprecatedTagName = "Deprecated";

        internal static Tag DeprecatedTag = new(DeprecatedTagName)
        {
            Behavior =
            {
                Order = 203,
                Color = ErrorColor,
                Icon = TextureContent.CreateContent("ovr_icon_deprecated.png", Utils.BuildingBlocksIcons,
                    HiddenTagName),
                Show = true,
                ShowOverlay = true,
                ToggleableVisibility = true,
                DefaultVisibility = false,
            }
        };

        private const string NewTagName = "New";

        internal static Tag NewTag = new(NewTagName)
        {
            Behavior =
            {
                Automated = true,
                Order = 202,
                Color = NewColor,
                Icon = TextureContent.CreateContent("ovr_icon_new.png", Utils.BuildingBlocksIcons,
                    NewTagName),
                Show = true,
                CanFilterBy = false,
                ShowOverlay = true,
            }
        };

#if OVRPLUGIN_TESTING
        internal static bool IsValidPackageId(string packageName)
        {
            const string pattern = @"^([a-z0-9]+(-[a-z0-9]+)*\.)+[a-z]{2,}(@([0-9]+\.){2}[0-9]+(-[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?(\+[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?)?$";
            var regex = new Regex(pattern, RegexOptions.IgnoreCase);
            return regex.IsMatch(packageName);
        }
#endif // OVRPLUGIN_TESTING


        private const string DocumentationUrl = "https://developer.oculus.com/documentation/unity/unity-buildingblocks-overview";

#if OVR_INTERNAL_CODE
        private const string WikiUrl =
            "https://www.internalfb.com/intern/wiki/Oculus/Vision/MixedReality/Presence_Platform_Frameworks/Unity/Smart_Building_Blocks/";
        private const string WorkplaceUrl = "https://fb.workplace.com/groups/339438274734616/permalink/584845710193870/";
        private const string FeedbackUrl = "https://www.fburl.com/ppt-feedback";
#endif

        internal static readonly Item Item = new()
        {
            Name = BlocksPublicName,
            Color = Styles.Colors.AccentColor,
            Icon = StatusIcon,
            InfoTextDelegate = ComputeInfoText,
            PillIcon = GetPillIcon,
            OnClickDelegate = OnStatusMenuClick,
            Order = 1,
            HeaderIcons = new List<Item.HeaderIcon>()
            {
                new()
                {
                    TextureContent = ConfigIcon,
                    Color = LightGray,
                    Action = BuildingBlocksWindow.ShowSettingsMenu
                },
                new()
                {
                    TextureContent = DocumentationIcon,
                    Color = LightGray,
                    Action = () => Application.OpenURL(DocumentationUrl)
                },
#if OVR_INTERNAL_CODE
                new()
                {
                    TextureContent = WikiIcon,
                    Color = InternalColor,
                    Action = () => Application.OpenURL(WikiUrl)
                },
                new()
                {
                    TextureContent = WorkplaceIcon,
                    Color = InternalColor,
                    Action = () => Application.OpenURL(WorkplaceUrl)
                },
                new()
                {
                    TextureContent = FeedbackIcon,
                    Color = InternalColor,
                    Action = () => Application.OpenURL(FeedbackUrl)
                }
#endif
            }
        };

        static Utils()
        {
            StatusMenu.RegisterItem(Item);

#if OVR_INTERNAL_CODE
            InitializeOnLoad.Register(RegisterBlockDataMenuItems);
            InitializeOnLoad.Register(RegisterInstallationRoutineMenuItems);
#endif // OVR_INTERNAL_CODE
        }

#if OVR_INTERNAL_CODE

        #region BlockDataMenuItem

        private static void RegisterBlockDataMenuItems()
        {
            var derivedTypes = AppDomain.CurrentDomain.GetAssemblies()
                .SelectMany(assembly => assembly.GetTypes())
                .Where(t => t.IsSubclassOf(typeof(BlockBaseData)) && !t.IsAbstract)
                .OrderBy(t => t switch
                {
                    _ when t == typeof(BlockData) => 0,
                    _ when t == typeof(BlockDownloaderData) => 1,
                    _ when t == typeof(InterfaceBlockData) => 2,
                    _ => 3
                }).ThenBy(t => t.Name);

            foreach (var type in derivedTypes)
            {
                var menuItemName = $"Assets/Create/Oculus/Block Data/{type.Name}";
                AddMenuItem(menuItemName, type);
            }
        }

        private static void RegisterInstallationRoutineMenuItems()
        {
            var derivedTypes = AppDomain.CurrentDomain.GetAssemblies()
                .SelectMany(assembly => assembly.GetTypes())
                .Where(t => t.IsSubclassOf(typeof(InstallationRoutine)) && !t.IsAbstract)
                .OrderBy(t => t.Name);

            foreach (var type in derivedTypes)
            {
                var menuItemName = $"Assets/Create/Oculus/Block Installation Routines/{type.Name}";
                AddMenuItem(menuItemName, type);
            }
        }

        private static void AddMenuItem(string menuPath, Type type)
        {
            var addMenuItemMethod = typeof(Menu).GetMethod("AddMenuItem", BindingFlags.NonPublic | BindingFlags.Static);
            addMenuItemMethod?.Invoke(null, new object[] { menuPath, null, null, -1, (Action)(() => CreateAssetOfType(type)), null });
        }

        internal static void CreateAssetOfType(Type type)
        {
            // Fetch the path of the currently selected folder in the Project tab
            var path = AssetDatabase.GetAssetPath(Selection.activeObject);
            if (string.IsNullOrEmpty(path))
            {
                path = "Assets"; // Default to the Assets folder if no path is found
            }
            else if (Path.GetExtension(path) != "")
            {
                path = path.Replace(Path.GetFileName(AssetDatabase.GetAssetPath(Selection.activeObject)), "");
            }

            var assetPathAndName = AssetDatabase.GenerateUniqueAssetPath(path + "/New" + type.Name + ".asset");

            var asset = ScriptableObject.CreateInstance(type);
            AssetDatabase.CreateAsset(asset, assetPathAndName);
            AssetDatabase.SaveAssets();
            EditorUtility.FocusProjectWindow();
            Selection.activeObject = asset;
        }

        #endregion

#endif // OVR_INTERNAL_CODE

        private static int ComputeNumberOfNewBlocks() =>
            BlockBaseData.Registry.Values.Count(data => !data.Hidden && data.Tags.Contains(NewTag));

        private static (string, Color?) ComputeInfoText()
        {
            var numberOfNewBlocks = ComputeNumberOfNewBlocks();
            if (numberOfNewBlocks > 0)
            {
                return (
                    $"There {OVREditorUtils.ChoosePlural(numberOfNewBlocks, "is", "are")} {numberOfNewBlocks} new {OVREditorUtils.ChoosePlural(numberOfNewBlocks, "block", "blocks")} available!",
                    NewColor);
            }

            var numberOfBlocks = GetBlocksInScene().Count;
            return (
                $"{numberOfBlocks} {OVREditorUtils.ChoosePlural(numberOfBlocks, "block", "blocks")} in current scene.",
                null);
        }

        private static (TextureContent, Color?) GetPillIcon()
        {
            if (ComputeNumberOfNewBlocks() > 0)
            {
                return (NewTag.Behavior.Icon, NewColor);
            }

            return (null, null);
        }

        private static void OnStatusMenuClick(Item.Origins origin)
        {
            BuildingBlocksWindow.ShowWindow(origin);
        }

        public static BlockData GetBlockData(this BuildingBlock block) => GetBlockData(block.blockId);

        public static BlockData GetBlockData(string blockId) => BlockBaseData.Registry[blockId] as BlockData;

        public static BuildingBlock GetBlock(this BlockData data)
        {
            return Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None)
                .FirstOrDefault(x => x.BlockId == data.Id);
        }

#if OVR_INTERNAL_CODE
        public static InstallationRoutine GetInstallationRoutine(this BuildingBlock block) =>
            GetInstallationRoutine(block.installationRoutineId);

        public static InstallationRoutine GetInstallationRoutine(string installationRoutineId) =>
            InstallationRoutine.Registry[installationRoutineId];
#endif

        public static BuildingBlock GetBlock(string blockId)
        {
            return GetBlockData(blockId)?.GetBlock();
        }

        public static List<BuildingBlock> GetBlocks(this BlockData data)
        {
            return Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Where(x => x.BlockId == data.Id)
                .ToList();
        }

        public static List<BuildingBlock> GetBlocks(string blockId)
        {
            return GetBlockData(blockId)?.GetBlocks();
        }

        public static List<T> GetBlocksWithType<T>() where T : Component
        {
            return Object.FindObjectsByType<T>(FindObjectsSortMode.None)
                .Where(controller => controller.GetComponent<BuildingBlock>() != null).ToList();
        }

        public static List<T> GetBlocksWithBaseClassType<T>() where T : Component
        {
            var objects = GetBlocksWithType<T>();
            return objects
                .Select(obj => obj.GetComponent<T>())
                .Where(component => component != null && component.GetType() == typeof(T))
                .ToList();
        }

        private static bool IsRequiredBy(this BlockData data, BlockData other)
        {
            if (data == null || other == null)
            {
                return false;
            }

            return data == other || other.Dependencies.Any(data.IsRequiredBy);
        }

        public static List<BuildingBlock> GetBlocksInScene()
        {
            return Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.InstanceID).ToList();
        }

        public static List<BuildingBlock> GetUsingBlocksInScene(this BlockData requiredData)
        {
            return Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Where(x =>
            {
                var data = x.GetBlockData();
                return requiredData != data && requiredData.IsRequiredBy(data);
            }).ToList();
        }

        public static List<BlockData> GetUsingBlockDatasInScene(this BlockData requiredData)
        {
            return requiredData.GetUsingBlocksInScene().Select(x => x.GetBlockData()).Distinct().ToList();
        }

        public static IEnumerable<BlockData> GetAllDependencies(this BlockData data) =>
            data.Dependencies
            .Where(dependency => dependency != null)
            .SelectMany(
                dependency => GetAllDependencies(dependency)
                .Concat(new[] { dependency })
            ).Distinct();

        public static void SelectBlockInScene(this BuildingBlock block)
        {
            Selection.activeGameObject = block.gameObject;
        }

        public static void SelectBlocksInScene(IEnumerable<GameObject> blockList)
        {
            Selection.objects = blockList.Cast<Object>().ToArray();
        }

        public static void SelectBlocksInScene(this BlockData blockData)
        {
            var blocksInScene = blockData.GetBlocks();

            if (blocksInScene.Count == 1)
            {
                SelectBlockInScene(blocksInScene[0]);
            }
            else if (blocksInScene.Count > 1)
            {
                SelectBlocksInScene(blocksInScene.Select(block => block.gameObject));
            }
        }

        public static void HighlightBlockInScene(this BuildingBlock block)
        {
            EditorGUIUtility.PingObject(block.gameObject);
        }

        public static int ComputeNumberOfBlocksInScene(this BlockData blockData)
        {
            return Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None)
                .Count(x => x.BlockId == blockData.Id);
        }

        public static T FindComponentInScene<T>() where T : Component
        {
            var scene = SceneManager.GetActiveScene();
            var rootGameObjects = scene.GetRootGameObjects();
            return rootGameObjects.FirstOrDefault(go => go.GetComponentInChildren<T>())?.GetComponentInChildren<T>();
        }



#if OVRPLUGIN_TESTING

        public static IEnumerable<BlockBaseData> PublicBlockData =>
            BlockBaseData.Registry.Values.Where(blockData =>
                !blockData.Hidden
                && !blockData.Tags.Contains(Utils.DeprecatedTag)
#if OVR_INTERNAL_CODE // BB_INTERFACE
                && blockData is not InterfaceBlockData
#else
                && !blockData.InternalBlock
#endif
            );

#if OVR_INTERNAL_CODE
        [CanBeNull]
#endif // OVR_INTERNAL_CODE
        public static T GetTestClassForBlockData<T>(string blockId) where T : class
        {
            var assemblies = AppDomain.CurrentDomain.GetAssemblies();

            foreach (var assembly in assemblies)
            {
                foreach (var type in assembly.GetTypes())
                {
                    if (!type.IsSubclassOf(typeof(T)) || type.IsAbstract)
                    {
                        continue;
                    }

                    var idProp = type.GetProperty("BlockId");
                    if (idProp == null || idProp.PropertyType != typeof(string))
                    {
                        continue;
                    }

                    // Here I'm assuming a parameterless constructor for simplicity
                    var instance = (T)Activator.CreateInstance(type);
                    if ((string)idProp.GetValue(instance) == blockId)
                    {
                        return instance;
                    }
                }
            }

            return null;
        }

#endif //OVR_INTERNAL_CODE

        public static TResult Let<TSource, TResult>(this TSource source, Func<TSource, TResult> func) => func(source);

        internal static bool HasDuplicates<T>(this IEnumerable<T> dependencies) =>
            dependencies
                .GroupBy(x => x)
                .Any(g => g.Count() > 1);

        /// <summary>
        /// Compare current Unity Editor version with target Unity Editor version.
        /// </summary>
        /// <remarks>
        /// Version format: Major.Minor.Build. Examples: 2022.3.2, 2023.3, 2021.3.2
        /// </remarks>
        /// <param name="target">Target version</param>
        /// <returns>
        /// returns -1 if current version is older than target
        /// returns 1 if current version is newer than target
        /// returns 0 if versions are same
        /// </returns>
        public static int CompareCurrentUnityEditorVersions(string target)
        {
            if (target == null)
                throw new ArgumentNullException();

            var pattern = @"[\.-]|(?<=\d)(?=[a-zA-Z])";
            var currentVersionParts = Regex.Split(Application.unityVersion, pattern)
                .Where(s => !String.IsNullOrEmpty(s))
                .Select(s =>
                {
                    int.TryParse(s, out var o);
                    return o;
                }).ToArray();

            var targetVersionParts = Regex.Split(target, pattern)
                .Where(s => !String.IsNullOrEmpty(s))
                .Select(s =>
                {
                    int.TryParse(s, out var o);
                    return o;
                }).ToArray();

            if (targetVersionParts.Length == 0)
            {
                throw new ArgumentException("Empty target version string.");
            }

            Version currentVersion = new Version(currentVersionParts[0], currentVersionParts[1], currentVersionParts[2]);
            Version targetVersion = new Version(targetVersionParts[0],
                targetVersionParts.Length > 1 ? targetVersionParts[1] : 0,
                targetVersionParts.Length > 2 ? targetVersionParts[2] : 0);

            return currentVersion.CompareTo(targetVersion);
        }
    }
}
