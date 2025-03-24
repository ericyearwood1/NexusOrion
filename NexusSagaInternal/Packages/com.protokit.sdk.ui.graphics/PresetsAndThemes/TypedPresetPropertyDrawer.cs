#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

[CustomPropertyDrawer(typeof(PresetsManager.TypedPreset))]
public class TypedPresetPropertyDrawer : PropertyDrawer {
    private SerializedProperty
        typedObject,
        assignedPreset,
        typedPresetsList;

    private GUIContent presetsListLabel = new GUIContent("Presets List");
    private Texture checkIconCache;
    private Texture checkIcon {
        get {
            if (checkIconCache == null) {
                checkIconCache = Resources.Load("check-32") as Texture;
            }
            return checkIconCache;
        }
    }

    public override float GetPropertyHeight(SerializedProperty property, GUIContent label) {
        var baseHeight = base.GetPropertyHeight(property, label);

        typedObject = property.FindPropertyRelative("TypedObject");
        assignedPreset = property.FindPropertyRelative("AssignedPreset");
        typedPresetsList = property.FindPropertyRelative("TypedPresets");

        var typedObjectHeight = EditorGUI.GetPropertyHeight(typedObject);
        var expanded = typedPresetsList.isExpanded;
        var typedPresetsListHeight = 0f;
        if (expanded) {
            typedPresetsListHeight = typedPresetsList.arraySize * (EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing) + EditorGUIUtility.standardVerticalSpacing;
        }
        var totalHeight = baseHeight + typedObjectHeight + typedPresetsListHeight + EditorGUIUtility.standardVerticalSpacing * 3;
        return totalHeight;
    }

    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
        EditorGUI.BeginProperty(position, label, property);

        var typedObjectRect = new Rect(position.x, position.y + EditorGUIUtility.standardVerticalSpacing, position.width, EditorGUIUtility.singleLineHeight);
        var foldoutRect = new Rect(position.x, typedObjectRect.y + typedObjectRect.height + EditorGUIUtility.standardVerticalSpacing, 60f, EditorGUIUtility.singleLineHeight);
        var typedPresetsListRect = new Rect(position.x, typedObjectRect.y + typedObjectRect.height + EditorGUIUtility.standardVerticalSpacing, position.width, EditorGUIUtility.singleLineHeight);
        var typedPresetsListElementRect = new Rect(position.x, typedPresetsListRect.y + typedPresetsListRect.height + EditorGUIUtility.standardVerticalSpacing, position.width, EditorGUIUtility.singleLineHeight);

        EditorGUI.ObjectField(typedObjectRect, typedObject, typeof(Component), new GUIContent("Apply to Component"));

        var currPreset = assignedPreset.objectReferenceValue;

        if (typedPresetsList.isExpanded = EditorGUI.Foldout(foldoutRect, typedPresetsList.isExpanded, presetsListLabel, true)) {
            EditorGUI.PropertyField(typedPresetsListRect, typedPresetsList.FindPropertyRelative("Array.size"), presetsListLabel);
            for (int i = 0; i < typedPresetsList.arraySize; i++) {
                var top = typedPresetsListElementRect.y + i * (EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing);
                var buttonWidth = 60f;
                var toggleRect = new Rect(typedPresetsListElementRect.x, top, 13f, 13f);
                var elementRect = new Rect(toggleRect.x + toggleRect.width + 2f, top, typedPresetsListElementRect.width - (buttonWidth + 2f) - (toggleRect.width + 2f), typedPresetsListElementRect.height);
                var buttonRect = new Rect(elementRect.x + elementRect.width + 2f, elementRect.y, buttonWidth, EditorGUIUtility.singleLineHeight);

                var preset = typedPresetsList.GetArrayElementAtIndex(i).objectReferenceValue;

                if (currPreset == preset) {
                    GUI.Box(toggleRect, checkIcon, EditorStyles.label);
                } else {
                    GUI.Box(toggleRect, "", EditorStyles.label);
                }

                EditorGUI.ObjectField(elementRect, typedPresetsList.GetArrayElementAtIndex(i), GUIContent.none);
                if (GUI.Button(buttonRect, "Apply")) {
                    var targets = property.serializedObject.targetObjects;
                    foreach (var target in targets) {
                        Undo.RecordObject(target, "Apply Preset");
                        var refObj = fieldInfo.GetValue(target);
                        PresetsManager.TypedPreset myDataClass = refObj as PresetsManager.TypedPreset;
                        if (refObj.GetType() == (typeof(List<PresetsManager.TypedPreset>))) {
                            var index = Convert.ToInt32(new string(property.propertyPath.Where(c => char.IsDigit(c)).ToArray()));
                            myDataClass = ((List<PresetsManager.TypedPreset>)refObj)[index];
                        }
                        myDataClass.ApplyTo(i);
                    }
                }
            }
        }
        EditorGUI.EndProperty();
    }
}
#endif
