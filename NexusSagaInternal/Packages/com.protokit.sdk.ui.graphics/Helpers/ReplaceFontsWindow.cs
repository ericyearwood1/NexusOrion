#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using TMPro;

public class ReplaceFontsWindow : EditorWindow
{
    [MenuItem("Replace Fonts", menuItem = "Protokit/Replace Fonts")]
    private static void Init() {
        var window = GetWindow<ReplaceFontsWindow>();
        window.titleContent = new GUIContent("Replace Fonts");
        window.minSize = new Vector2(600f, 300f);
        window.maxSize = new Vector2(600f, 300f);

        window.Show();
    }

    private TMP_FontAsset fontAsset;
    // styles
    private float padding = 4;
    private GUIStyle buttonStyle;

    private void OnGUI() {
        if (buttonStyle == null) {
            buttonStyle = EditorStyles.miniButton;
            buttonStyle.fixedHeight = 0;
            buttonStyle.fontSize = 12;
            buttonStyle.margin = new RectOffset(0, 0, 0, 0);
        }

        GUILayout.Space(padding * 2);

        using (new EditorGUILayout.HorizontalScope()) {
            GUILayout.Space(padding);

            EditorGUILayout.LabelField("Replace all fonts in the Scene with ", GUILayout.Width(200));
            fontAsset = EditorGUILayout.ObjectField(fontAsset, typeof(TMP_FontAsset), false, GUILayout.Width(200)) as TMP_FontAsset;
            if (fontAsset == null) {
                using (new EditorGUI.DisabledScope(true)) {
                    EditorGUILayout.LabelField("default: LiberationSans");
                }
            }

            GUILayout.Space(padding);
        }

        GUILayout.Space(padding);

        using (new EditorGUILayout.HorizontalScope()) {
            GUILayout.Space(padding);

            if (GUILayout.Button("Replace Fonts", GUILayout.Height(30))) {
                ReplaceFont();
            }

            GUILayout.Space(padding);
        }

    }

    public void ReplaceFont() {
        var rootGameObjects = EditorSceneManager.GetActiveScene().GetRootGameObjects();
        foreach (var go in rootGameObjects) {
            var texts = go.GetComponentsInChildren<TMP_Text>(true);
            foreach (var t in texts) {
                t.enabled = false;
                t.font = fontAsset;
                t.enabled = true;
                t.SetAllDirty();
            }
        }
    }
}
#endif
