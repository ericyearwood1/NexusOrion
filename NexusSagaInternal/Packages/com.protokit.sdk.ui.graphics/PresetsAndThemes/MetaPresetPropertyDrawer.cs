#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

[CustomPropertyDrawer(typeof(PresetsManager.MetaPreset))]
public class MetaPresetPropertyDrawer : PropertyDrawer {
    private SerializedProperty
        nameProp,
        editableNameProp;

    private Texture removeIconCache;
    private Texture removeIcon {
        get {
            if (removeIconCache == null) {
                removeIconCache = Resources.Load("delete-24") as Texture;
            }
            return removeIconCache;
        }
    }

    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
        nameProp = property.FindPropertyRelative("Name");
        editableNameProp = property.FindPropertyRelative("EditableName");

        EditorGUI.BeginProperty(position, label, property);

        var buttonWidth = 60f;
        var namePropRect = new Rect(position.x, position.y + EditorGUIUtility.standardVerticalSpacing, position.width - buttonWidth - 8f - 13f, EditorGUIUtility.singleLineHeight);
        var presetsAssemblyPropRect = new Rect(namePropRect.x + namePropRect.width + 2f, position.y, position.width - namePropRect.width - 2f, EditorGUIUtility.singleLineHeight);

        var applyButtonRect = new Rect(namePropRect.x + namePropRect.width + 2f, position.y + EditorGUIUtility.standardVerticalSpacing, buttonWidth, EditorGUIUtility.singleLineHeight);
        var deleteIconRect = new Rect(applyButtonRect.x + applyButtonRect.width + 2f, namePropRect.y + 2f, 13f, 13f);

        if (editableNameProp.boolValue == true) {
            nameProp.stringValue = EditorGUI.TextField(namePropRect, nameProp.stringValue);
        }
        else {
            EditorGUI.LabelField(namePropRect, nameProp.stringValue);
        }

        if (GUI.Button(applyButtonRect, "Apply")) {
            var targets = property.serializedObject.targetObjects;
            foreach (var target in targets) {
                var refObj = fieldInfo.GetValue(target);
                PresetsManager.MetaPreset myDataClass = refObj as PresetsManager.MetaPreset;
                if (refObj.GetType() == (typeof(List<PresetsManager.MetaPreset>))) {
                    var index = Convert.ToInt32(new string(property.propertyPath.Where(c => char.IsDigit(c)).ToArray()));
                    myDataClass = ((List<PresetsManager.MetaPreset>)refObj)[index];
                }
                myDataClass.ApplyAll();
            }
        }
        if (GUI.Button(deleteIconRect, removeIcon, EditorStyles.label)) {
            var targets = property.serializedObject.targetObjects;
            foreach (var target in targets) {
                Undo.RecordObject(target, "Remove Meta Preset");
                var refObj = fieldInfo.GetValue(target);
                PresetsManager.MetaPreset myDataClass = refObj as PresetsManager.MetaPreset;
                if (refObj.GetType() == (typeof(List<PresetsManager.MetaPreset>))) {
                    var index = Convert.ToInt32(new string(property.propertyPath.Where(c => char.IsDigit(c)).ToArray()));
                    myDataClass = ((List<PresetsManager.MetaPreset>)refObj)[index];
                }
                (refObj as List<PresetsManager.MetaPreset>).Remove(myDataClass);
            }
            return;
        }

        EditorGUI.EndProperty();
    }
}
#endif
