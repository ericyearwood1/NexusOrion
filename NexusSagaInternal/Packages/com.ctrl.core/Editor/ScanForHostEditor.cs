
using UnityEngine;
using UnityEngine.Events;
using UnityEditor;

using System;
using System.Collections.Generic;
using System.Linq;

using CTRL.ClientBehaviors;

namespace CTRL.Inspector
{
  [CustomEditor(typeof(ScanForHost))]
  public class ScanForHostEditor : Editor
  {
#if !UNITY_WEBGL

    private ScanForHost component
    {
      get { return target as ScanForHost; }
    }

    private string status;
    private List<string> potentialHosts = new List<string>();

    public override void OnInspectorGUI()
    {
      DrawDefaultInspector();

      var text = !component.IsScanning ? "Scan" : "Cancel Scan";
      if (GUILayout.Button(text))
      {
        if (!component.IsScanning)
        {
          Scan();
        }
        else
        {
          component.CancelScan();
        }
      }

      TextFieldReadOnly("Status", status);
      for (int i = 0; i < potentialHosts.Count; i++)
      {
        string host = potentialHosts[i];
        string label = "Potential Host" + (i == 0 ? " (default)" : "");
        TextFieldReadOnly(label, host);
      }

      GUI.enabled = true;
    }

    private async void Scan()
    {
      if (component.IsScanning)
      {
        return;
      }

      potentialHosts.Clear();

      var scanTask = component.Scan();
      UpdateStatus();

      component.OnPotentialHostFound.AddListener(OnPotentialHostFound);
      potentialHosts = await scanTask;
      component.OnPotentialHostFound.RemoveListener(OnPotentialHostFound);

      if (potentialHosts.Count > 0)
      {
        component.client.Host = potentialHosts[0];
      }

      UpdateStatus();
      Repaint();
    }

    void OnPotentialHostFound(string host)
    {
      potentialHosts.Add(host);
      UpdateStatus();
    }

    private void UpdateStatus()
    {
      int hostCount = potentialHosts.Count;
      status = component.IsScanning ?
          $"Scanning [found {hostCount} potential hosts so far] ..." :
          (hostCount > 0) ?
              $"Found {potentialHosts[0]} (out of {hostCount} potential hosts)." :
              "No hosts found!";
    }

    void TextFieldReadOnly(string label, string text)
    {
      EditorGUILayout.BeginHorizontal();
      EditorGUILayout.LabelField(label, GUILayout.Width(EditorGUIUtility.labelWidth - 4));
      EditorGUILayout.SelectableLabel(text, EditorStyles.textField, GUILayout.Height(EditorGUIUtility.singleLineHeight));
      EditorGUILayout.EndHorizontal();
    }
#endif
  }
}
