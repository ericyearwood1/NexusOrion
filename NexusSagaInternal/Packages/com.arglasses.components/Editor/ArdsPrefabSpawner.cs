#if UNITY_EDITOR
using ARGlasses.Interaction;
using UnityEditor;
using UnityEngine;

internal static class ArdsPrefabSpawner
{
    /* Instantiate components from GameObject menu */

    [MenuItem("GameObject/ARDS/Button", priority = 10)]
    private static void ARDS_Button(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("4ff56cc5fb0ad024eafc7dbcad4617c8", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/Slider", priority = 10)]
    private static void ARDS_Slider(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("58cc072b6bec60245a547e8cd8bd9e22", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/Toggle", priority = 10)]
    private static void ARDS_Toggle(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("879a919437273ea49846899ced19809d", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/TileButton", priority = 10)]
    private static void ARDS_TileButton(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("5cf47d77c4ed64e4997dc91d2d85d5da", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/ExperienceButton", priority = 10)]
    private static void ARDS_ExpButton(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("bfcb40375884944499931b3b294dd615", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/ListItem", priority = 10)]
    private static void ARDS_ListItem(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("683c25c0e7e49854f9c9e3739f224065", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/Primitives/Icon", priority = 10)]
    private static void ARDS_Icon(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("9a3cf5a05ec6217479046017f94577ea", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/Primitives/Avatar", priority = 10)]
    private static void ARDS_Avatar(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("8e01579cb5816f64db716ba690dc4509", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/Primitives/Text", priority = 10)]
    private static void ARDS_Text(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("d0680e04d6f9ce94cbe6498544f34c38", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/Primitives/Panel", priority = 10)]
    private static void ARDS_Panel(MenuCommand menuCommand)
    {
        var prefabGo = UILibraryUtility.Instantiate("1bbeac0f3b9646643b32dc766987dc9c", menuCommand, true);
        PrefabUtility.UnpackPrefabInstance(prefabGo, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
    }

    [MenuItem("GameObject/ARDS/Primitives/Panel - Interactive", priority = 11)]
    private static void ARDS_PanelInteractive(MenuCommand menuCommand)
    {
        var prefabGo = UILibraryUtility.Instantiate("3080bdb3ebbd3b9499aa702be5e077b9", menuCommand, true);
        PrefabUtility.UnpackPrefabInstance(prefabGo, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
    }

    [MenuItem("GameObject/ARDS/AlertDialogue", priority = 10)]
    private static void ARDS_AlertDialogue(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("73d63397587f6394e98986611a52036e", menuCommand, true);
    }

    [MenuItem("GameObject/ARDS/Navigation/Tab Bar", priority = 10)]
    private static void ARDS_SegmentedToggleGroup(MenuCommand menuCommand)
    {
        var prefabGo = UILibraryUtility.Instantiate("461511e98681f674b96ac0c72ed396eb", menuCommand, true);
        PrefabUtility.UnpackPrefabInstance(prefabGo, PrefabUnpackMode.OutermostRoot, InteractionMode.AutomatedAction);
    }

    [MenuItem("GameObject/ARDS/Launcher/App Button", priority = 10)]
    private static void ARDS_LauncherAppButton(MenuCommand menuCommand)
    {
        var prefabGo = UILibraryUtility.Instantiate("4771e3e3d055388478b87ef49b69c10e", menuCommand, true);
        PrefabUtility.UnpackPrefabInstance(prefabGo, PrefabUnpackMode.OutermostRoot, InteractionMode.AutomatedAction);
    }

    [MenuItem("GameObject/ARDS/Tooltip", priority = 10)]
    private static void ARDS_Tooltip(MenuCommand menuCommand)
    {
        UILibraryUtility.Instantiate("b553e4c24cf27ee4284c7d52475d976b", menuCommand, true);
    }
    
    [MenuItem("GameObject/ARDS/Navigation/Overflow Menu", priority = 10)]
    private static void ARDS_OverflowMenu(MenuCommand menuCommand)
    {
        var prefabGo = UILibraryUtility.Instantiate("e994662d06a559143b2864f70d695cac", menuCommand, true);
        PrefabUtility.UnpackPrefabInstance(prefabGo, PrefabUnpackMode.OutermostRoot, InteractionMode.AutomatedAction);
    }


    public static GameObject InstantiateAtRoot(string guid)
    {
        var prefabPath = AssetDatabase.GUIDToAssetPath(guid);
        var o = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);

        GameObject instance;
        if (!CommonComponentsPreferences.InstantiateComponentsAsPrefabs)
        {
            instance = GameObject.Instantiate(o);
            instance.name = o.name;
        }
        else
        {
            instance = (GameObject)PrefabUtility.InstantiatePrefab(o);
        }

        if (instance == null)
        {
            return instance;
        }

        instance.name = o.name;

        return instance;
    }
    // <NEW-COMPONENT-CREATE>
}
#endif
