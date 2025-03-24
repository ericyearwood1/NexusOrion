using System;

using UnityEngine;
using UnityEditor;

using CTRL.Utils.Logging;

namespace CTRL.Inspector
{
  [CustomPropertyDrawer(typeof(CTRLLogger))]
  public class CTRLLoggerDrawer : PropertyDrawer
  {
    private static readonly string LOG_LEVEL_LABEL = "Log Level";
    private static readonly string TAG_LABEL = "Tag";
    private static readonly string WDP_GUID_LABEL = "WDP GUID";

    bool expanded = false;

    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {
      label = expanded ? label : new GUIContent($"Logger ({LOG_LEVEL_LABEL})");

      expanded = EditorGUI.Foldout(position, expanded, label);

      if (expanded)
      {
        EditorGUI.indentLevel++;
        EditorGUILayout.PropertyField(property.FindPropertyRelative("_filterLogType"), new GUIContent(LOG_LEVEL_LABEL));
        EditorGUILayout.PropertyField(property.FindPropertyRelative("_tag"), new GUIContent(TAG_LABEL));
        EditorGUILayout.PropertyField(property.FindPropertyRelative("_guid"), new GUIContent(WDP_GUID_LABEL));
        EditorGUI.indentLevel--;
      }
      else
      {
        float dx = EditorGUIUtility.labelWidth + 2;
        position.x += dx;
        position.width -= dx;
        EditorGUI.PropertyField(position, property.FindPropertyRelative("_filterLogType"), GUIContent.none);
      }
    }
  }
}
