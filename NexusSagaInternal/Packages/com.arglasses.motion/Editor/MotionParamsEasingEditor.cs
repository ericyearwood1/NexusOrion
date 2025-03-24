using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Interaction.Motion
{
    [CustomPropertyDrawer(typeof(MotionParamsEasing))]
    public class MotionParamsEasingEditor : PropertyDrawer
    {
        // Object
        private SerializedProperty _motionParamsProperty;
        private const string EASE_PROP = "_easing";
        private const string DUR_PROP = "_duration";
        private SerializedProperty _eProp, _dProp;

        // Layout attributes
        int _linesExpanded = 3;
        
        // Presets
        int _selectedPreset;
        
        public override void OnGUI(Rect rect, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(rect, label, property);

            GetPropertyValues(property);
            DrawFoldoutBox(rect, label);
            DrawProperties(rect);

            EditorGUI.EndProperty();
        }

        private void GetPropertyValues(SerializedProperty property)
        {
            _motionParamsProperty = property;
            _eProp = property.FindPropertyRelative(EASE_PROP);
            _dProp = property.FindPropertyRelative(DUR_PROP);
        }

        private void DrawFoldoutBox(Rect rect, GUIContent label)
        {
            Rect foldOutBox = new Rect(rect.position, new Vector2(rect.size.x, EditorGUIUtility.singleLineHeight));
            _motionParamsProperty.isExpanded = EditorGUI.Foldout(foldOutBox, _motionParamsProperty.isExpanded, label);
        }

        void DrawProperties(Rect rect)
        {
            if (_motionParamsProperty.isExpanded)
            {
                EditorGUI.indentLevel = 1;
                DrawEasingWithPreset(rect, 1);
                
                EditorGUI.indentLevel = 1;
                EditorGUI.PropertyField(GetExpandedPropertyLineRect(rect, 2), _dProp);
            }
            else
            {
                EditorGUI.indentLevel = 0;
                Rect controlsRect = GetControlsRectMinusLabel(rect);
                (Rect, Rect) containers = SplitRect(controlsRect, 0.5f);
                float labelWidthCache = EditorGUIUtility.labelWidth;
                EditorGUIUtility.labelWidth = containers.Item1.size.x * 0.33f;
                EditorGUI.PropertyField(containers.Item1, _dProp, new GUIContent("Duration"));
                EditorGUIUtility.labelWidth = labelWidthCache;
                _eProp.animationCurveValue = EditorGUI.CurveField(containers.Item2, _eProp.animationCurveValue);
            }
        }

        void DrawEasingWithPreset(Rect rect, int lineIncrement)
        {
            Rect containerRect = GetExpandedPropertyLineRect(rect, lineIncrement);
            EditorGUI.LabelField(GetLabelsOnlyRect(containerRect), new GUIContent("Easing"));
            
            EditorGUI.indentLevel = 0;
            Rect controlsRect = GetControlsRectMinusLabel(containerRect);
            (Rect, Rect) splitRect = SplitRect(controlsRect, .33f);
            _eProp.animationCurveValue = EditorGUI.CurveField(splitRect.Item1, _eProp.animationCurveValue);
            DrawPresetSelector(splitRect.Item2, 2);
        }

        void DrawPresetSelector(Rect rect, int lineIncrement)
        {
            EditorGUI.indentLevel = 0;
            PropertyInfo[] properties = typeof(Easing).GetProperties();
            GUIContent[] options = new GUIContent[properties.Length + 1];

            for (int i = 0; i < options.Length; i++)
            {
                if (i == 0)
                {
                    options[i] = new GUIContent("...");
                }
                else
                {
                    options[i] = new GUIContent(properties[i-1].Name);
                }
            }

            _selectedPreset = EditorGUI.Popup(rect, _selectedPreset, options);
            _eProp.animationCurveValue =
                _selectedPreset == 0 ? _eProp.animationCurveValue : (AnimationCurve)properties[_selectedPreset-1].GetValue(new Easing());
        }
        
        private Rect GetExpandedPropertyLineRect(Rect defaultRect, int lineIncrement)
        {
            float yOffset = lineIncrement * EditorGUIUtility.singleLineHeight;
            Vector2 position = new Vector2(defaultRect.position.x, defaultRect.position.y + yOffset);
            Vector2 size = new Vector2(defaultRect.size.x, EditorGUIUtility.singleLineHeight);
            Rect adjRect = new Rect(position, size);

            return adjRect;
        }

        private Rect GetLabelsOnlyRect(Rect parentRect)
        {
            return new Rect(parentRect.position,
                new Vector2(EditorGUIUtility.labelWidth, EditorGUIUtility.singleLineHeight));
        }
        
        private Rect GetControlsRectMinusLabel(Rect parentRect)
        {
            return new Rect(parentRect.position + (Vector2.right * EditorGUIUtility.labelWidth),
                new Vector2(parentRect.size.x - EditorGUIUtility.labelWidth, EditorGUIUtility.singleLineHeight));
        }
        
        private (Rect, Rect) SplitRect(Rect parentRect, float sizeRatio)
        {
            Rect leftRect = new Rect(parentRect.position, new Vector2(parentRect.size.x * sizeRatio, EditorGUIUtility.singleLineHeight));
            Rect rightRect = new Rect(parentRect.position + (Vector2.right * leftRect.size.x), new Vector2(parentRect.size.x * (1-sizeRatio), EditorGUIUtility.singleLineHeight));
            return (leftRect, rightRect);
        }
        
        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            if (property.isExpanded)
            {
                return EditorGUIUtility.singleLineHeight * _linesExpanded;
            }
            else
            {
                return EditorGUIUtility.singleLineHeight;
            }
        }
    }
}
