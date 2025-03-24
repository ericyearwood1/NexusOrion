using UnityEditor;
using UnityEngine;

using CTRL.Schemes;

namespace CTRL.Inspector
{
  [CustomEditor(typeof(Pinch))]
  public class PinchEditor : Editor
  {
    public override void OnInspectorGUI()
    {
      var element = target as Pinch;

      DrawDefaultInspector();

      EditorGUILayout.BeginVertical("Box", GUILayout.ExpandHeight(true));
      if (element.IsConnected)
      {
        EditorGUILayout.LabelField(string.Format("Index: {0}", element.GetFinger(Finger.Index).IsPinched ? "Pinched" : "Released"));
        EditorGUILayout.LabelField(string.Format("Middle: {0}", element.GetFinger(Finger.Middle).IsPinched ? "Pinched" : "Released"));
      }
      else
      {
        EditorGUILayout.LabelField("[disconnected]");
      }
      EditorGUILayout.EndVertical();
    }

    // TODO: any performance concerns?
    public override bool RequiresConstantRepaint()
    {
      return true;
    }
  }
}
