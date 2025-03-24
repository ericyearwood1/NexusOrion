using UnityEngine;
using UnityEditor;

using System;

using CTRL.ClientBehaviors;

namespace CTRL.Inspector
{
  [CustomEditor(typeof(LoadConfigFile))]
  public class LoadConfigFileEditor : Editor
  {

    private string host;
    private int port;

    private LoadConfigFile component
    {
      get { return target as LoadConfigFile; }
    }

    public override void OnInspectorGUI()
    {
      DrawDefaultInspector();

      if (GUILayout.Button("Load"))
      {
        component.Load();
        host = component.client.Host;
        port = component.client.Port;
      }

      TextFieldReadOnly("CTRL-R Host", host);
      TextFieldReadOnly("CTRL-R Port", port.ToString());

      GUI.enabled = true;
    }

    void TextFieldReadOnly(string label, string text)
    {
      EditorGUILayout.BeginHorizontal();
      EditorGUILayout.LabelField(label, GUILayout.Width(EditorGUIUtility.labelWidth - 4));
      EditorGUILayout.SelectableLabel(text, EditorStyles.textField, GUILayout.Height(EditorGUIUtility.singleLineHeight));
      EditorGUILayout.EndHorizontal();
    }
  }
}
