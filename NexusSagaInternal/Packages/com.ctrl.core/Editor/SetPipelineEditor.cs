using UnityEngine;
using UnityEditor;

using CTRL.ClientBehaviors;

namespace CTRL.Inspector
{
  [CustomEditor(typeof(SetPipeline))]
  public class SetPipelineEditor : Editor
  {
    public override void OnInspectorGUI()
    {
      var component = target as SetPipeline;

      DrawDefaultInspector();

      GUI.enabled = component.Config != null;
      if (GUILayout.Button("Send now"))
      {
        component.SendConfig();
      }
      GUI.enabled = true;
    }
  }
}
