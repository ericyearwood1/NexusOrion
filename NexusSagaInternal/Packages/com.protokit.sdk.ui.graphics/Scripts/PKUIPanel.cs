namespace ProtoKit.UI {
    using GraphicBase;
    using UnityEngine;
#if UNITY_EDITOR
    using UnityEditor;
#endif

    [ExecuteAlways]
    [AddComponentMenu("OCUI/Protokit UI Panel")]
    public class PKUIPanel : RoundedRect {

        #region Editor

#if UNITY_EDITOR
        [CanEditMultipleObjects]
        [CustomEditor(typeof(PKUIPanel))]
        public class ProtokitUIPanelEditor : RoundedRectEditor {
            private SerializedProperty colorPresetProp;

            protected override void OnEnable() {
                base.OnEnable();
                drawExtraSettings = true;
                showExtraSettings = true;
            }

            protected override void OnDisable() {
                base.OnDisable();
            }

            public override void OnInspectorGUI() {
                EditorGUI.BeginChangeCheck();

                using (new GUILayout.VerticalScope()) {
                    base.OnInspectorGUI();
                }

                if (EditorGUI.EndChangeCheck()) {
                    serializedObject.ApplyModifiedProperties();
                    foreach (var t in targets) {
                        (t as PKUIPanel).UpdateProperties();
                        EditorUtility.SetDirty(t);
                    }
                }
            }
        }
#endif

        #endregion
    }
}
