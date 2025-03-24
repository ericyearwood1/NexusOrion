using UnityEditor;
using System.Linq;

namespace CTRL.Inspector
{
  [CustomEditor(typeof(CTRLClient))]
  public class ClientEditor : Editor
  {
    protected bool showState = false;

    public override void OnInspectorGUI()
    {
      var client = target as CTRLClient;

      DrawDefaultInspector();

      showState = EditorGUILayout.Foldout(showState, "State", true);
      if (showState)
      {
        EditorGUILayout.BeginVertical("Box");

        if (client.State == ConnectionState.Connected)
        {
          EditorGUILayout.LabelField("Connected!");
        }
        else
        {
          EditorGUILayout.LabelField("Disconnected");
        }

        if (client.Streams.Count() > 0)
        {
          EditorGUILayout.Space();
          foreach (var stream in client.Streams)
          {
            EditorGUILayout.LabelField($"{stream.StreamName}: {stream.State}");
          }
        }

        EditorGUILayout.EndVertical();
      }
    }
  }
}
