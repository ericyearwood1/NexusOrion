using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditorInternal;
using UnityEditor.Presets;
#endif

public class PresetsManager : MonoBehaviour
{
    private void OnEnable() {
        // required to have the enable/disable behavior.
        // this MonoBehavior will be unused at runtime.
    }

#if UNITY_EDITOR
    [System.Serializable]
    public class MetaPreset {
        public string Name = "New Preset";
        public bool EditableName = false;

        [System.Serializable]
        public class PresetsGroup {
            public TypedPreset Key;
            public Preset Value;
        }

        [SerializeField]
        public List<PresetsGroup> PresetsAssembly;

        public void ApplyAll() {
            foreach (var p in PresetsAssembly) {
                Undo.RecordObject(p.Key.TypedObject, "Apply Meta Preset");
                p.Key.SetAssignedPreset(p.Value);
            }
        }
    }
    [SerializeField] public List<MetaPreset> MetaPresets;

    public void SaveToMetaPresets() {
        var metaPreset = new MetaPreset();
        metaPreset.PresetsAssembly = new List<MetaPreset.PresetsGroup>();
        foreach (var p in Presets) {
            metaPreset.PresetsAssembly.Add(new MetaPreset.PresetsGroup() { Key = p, Value = p.AssignedPreset });
        }
        Undo.RecordObject(this, "Add Meta Preset");
        MetaPresets.Add(metaPreset);
    }

    [System.Serializable]
    public class TypedPreset {
        public Object TypedObject;
        public List<Preset> TypedPresets;
        public Preset AssignedPreset;

        public bool ApplyTo(int index) {
            Undo.RecordObject(TypedObject, "Apply Preset");
            var preset = TypedPresets[index];
            if (!preset.CanBeAppliedTo(TypedObject)) {
                return false;
            }

            AssignedPreset = preset;
            return ApplyAssignedPreset();
        }

        public bool SetAssignedPreset(Preset p) {
            AssignedPreset = p;
            return ApplyAssignedPreset();
        }

        private bool ApplyAssignedPreset() {
            if (AssignedPreset == null) return false;
            return AssignedPreset.ApplyTo(TypedObject);
        }
    }
    [SerializeField] public List<TypedPreset> Presets;

    [CanEditMultipleObjects]
    [CustomEditor(typeof(PresetsManager))]
    public class Editor : UnityEditor.Editor {
        private Texture editIconCache;
        private Texture editIcon {
            get {
                if (editIconCache == null) {
                    editIconCache = Resources.Load("edit-32") as Texture;
                }
                return editIconCache;
            }
        }

        private SerializedProperty metaPresetsProp;
        private SerializedProperty presetsProp;
        private ReorderableList list;
        private bool canEditNames = false;

        private void OnEnable() {
            metaPresetsProp = serializedObject.FindProperty(nameof(PresetsManager.MetaPresets));
            presetsProp = serializedObject.FindProperty(nameof(PresetsManager.Presets));

            list = new ReorderableList(serializedObject, presetsProp, true, true, true, true);
            list.drawElementCallback = DrawListElement;
            list.drawHeaderCallback = DrawListHeader;
            list.elementHeightCallback = ElementHeight;

            canEditNames = false;
        }

        private void OnDestroy() {
            metaPresetsProp.Dispose();
            presetsProp.Dispose();
        }

        public override void OnInspectorGUI() {
            // allows pinging the script
            using (new EditorGUI.DisabledScope(true)) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("m_Script"));
            }

            using (var check = new EditorGUI.ChangeCheckScope()) {
                using (new EditorGUILayout.HorizontalScope()) {
                    EditorGUILayout.LabelField("Presets", EditorStyles.boldLabel, GUILayout.Width(54f));
                    if (GUILayout.Button(editIcon, EditorStyles.label, GUILayout.Width(14f), GUILayout.Height(14f))) {
                        canEditNames = !canEditNames;
                    }
                }
                EditorGUI.indentLevel++;
                var sizeOfList = metaPresetsProp.arraySize;
                for (int i = 0; i < sizeOfList; ++i) {
                    var element = metaPresetsProp.GetArrayElementAtIndex(i);
                    element.FindPropertyRelative("EditableName").boolValue = canEditNames;
                    EditorGUILayout.PropertyField(element);
                }
                EditorGUI.indentLevel--;

                if (sizeOfList > 0) {
                    EditorGUILayout.Space();
                }

                EditorGUI.indentLevel++;
                presetsProp.isExpanded = EditorGUILayout.Foldout(presetsProp.isExpanded, "Configuration", true);
                EditorGUI.indentLevel--;
                if (presetsProp.isExpanded) {
                    list.DoLayoutList();
                    if (GUILayout.Button("Add To Presets")) {
                        foreach (var t in targets) {
                            (t as PresetsManager).SaveToMetaPresets();
                            EditorUtility.SetDirty(t);
                        }
                        canEditNames = true;
                    }
                }

                if (check.changed) {
                    serializedObject.ApplyModifiedProperties();
                }
            }
        }

        private void DrawListHeader(Rect rect) {
            EditorGUI.LabelField(rect, "Presets");
        }

        private void DrawListElement(Rect rect, int index, bool isActive, bool isFocused) {
            if (presetsProp.isExpanded) {
                var element = list.serializedProperty.GetArrayElementAtIndex(index);
                EditorGUI.PropertyField(new Rect(rect.x, rect.y, rect.width, EditorGUI.GetPropertyHeight(element)), element);
            }
        }

        private float ElementHeight(int index) {
            var element = list.serializedProperty.GetArrayElementAtIndex(index);
            var listHeight = EditorGUI.GetPropertyHeight(element);

            var baseHeight = listHeight;
            return baseHeight;
        }
    }
#endif
}
