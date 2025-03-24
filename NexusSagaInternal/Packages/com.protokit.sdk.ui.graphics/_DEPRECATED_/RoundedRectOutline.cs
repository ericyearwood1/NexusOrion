using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class RoundedRectOutline : MonoBehaviour {

#if UNITY_EDITOR
    [CustomEditor(typeof(RoundedRectOutline))]
    public class RoundedRectOutlineEditor : Editor {

        public override void OnInspectorGUI() {
            base.OnInspectorGUI();

            EditorGUILayout.HelpBox("This component has been deprecated and is no longer required.  You can delete it and the associated GameObject, just check to see if you have made any additional modifications you may want to keep.", MessageType.Warning);
        }
    }
#endif
}
