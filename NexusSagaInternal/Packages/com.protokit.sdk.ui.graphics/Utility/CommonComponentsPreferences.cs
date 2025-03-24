using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public static class CommonComponentsPreferences {

    private const string INSTANTIATE_COMPONENTS_AS_PREFABS_KEY = "OsigTools_Components_InstantiateComponentsAsPrefabs";
    private const string PANEL_CORNERS_SUBDIVISIONS_KEY = "OsigTools_Components_PanelCornersSubdivisions";

    public static bool EnabledPanelDepth {
        get {
#if OSIG_PANEL_DEPTH
            return true;
#else
            return false;
#endif
        }
    }

    public static int PanelCornersSubdivisions {
        get {
#if UNITY_EDITOR
            return EditorPrefs.GetInt(PANEL_CORNERS_SUBDIVISIONS_KEY, defaultValue: 8);
#else
            return PlayerPrefs.GetInt(PANEL_CORNERS_SUBDIVISIONS_KEY, defaultValue: 8);
#endif
        }
        set {
#if UNITY_EDITOR
            EditorPrefs.SetInt(PANEL_CORNERS_SUBDIVISIONS_KEY, value);
#endif
            PlayerPrefs.SetInt(PANEL_CORNERS_SUBDIVISIONS_KEY, value);
        }
    }

#if UNITY_EDITOR
    public static bool InstantiateComponentsAsPrefabs {
        get => EditorPrefs.GetBool(INSTANTIATE_COMPONENTS_AS_PREFABS_KEY, defaultValue: true);
        set => EditorPrefs.SetBool(INSTANTIATE_COMPONENTS_AS_PREFABS_KEY, value);
    }

    [SettingsProvider]
    private static SettingsProvider CreateCustomSettingsProvider() {
        return new SettingsProvider("Preferences/OSIG/UI Components", SettingsScope.User) {
            label = "OCUI Components",
            keywords = new HashSet<string>() { "OSIG", "Tools", "Components", "OCUI" },
            guiHandler = (ctx) => {
                EditorGUIUtility.labelWidth = 200;

                EditorGUILayout.Space();
                EditorGUILayout.LabelField("General", EditorStyles.boldLabel);

                using (var check = new EditorGUI.ChangeCheckScope()) {
                    bool newValue = EditorGUILayout.Toggle("Instantiate as Prefabs", InstantiateComponentsAsPrefabs);
                    if (check.changed) {
                        InstantiateComponentsAsPrefabs = newValue;
                    }
                }

                if (EnabledPanelDepth) {
                    EditorGUILayout.Space();
                    EditorGUILayout.LabelField("Extruded Panel", EditorStyles.boldLabel);
                    using (var check = new EditorGUI.ChangeCheckScope()) {
                        int newValue = EditorGUILayout.IntField("Corners Subdivisions", PanelCornersSubdivisions);
                        if (check.changed) {
                            PanelCornersSubdivisions = newValue;
                        }
                    }
                }
            }
        };
    }
#endif

        }
