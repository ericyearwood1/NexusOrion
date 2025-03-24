#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ArgAvatar))]
    [CanEditMultipleObjects]
    public class ArgAvatarEditor : Editor
    {
        private ArgAvatar _argAvatar;

        private void OnEnable()
        {
            _argAvatar = (ArgAvatar)target;

            if (!EditorApplication.isPlaying)
            {
                SwitchPrefab();
            }
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            if (_argAvatar.View == null)
            {
                SwitchPrefab();
                base.OnInspectorGUI();
                return;
            }

            EditorGUI.BeginChangeCheck();
            AvatarStyle currentStyle = _argAvatar.ViewModel.Style;
            currentStyle.Size = (AvatarSize)EditorGUILayout.EnumPopup("Size", currentStyle.Size);
            if (EditorGUI.EndChangeCheck())
            {
                _argAvatar.ViewModel.Style = currentStyle;
                SwitchPrefab();
            }


            _argAvatar.ViewModel.AvatarImage = (Sprite)EditorGUILayout.ObjectField("Avatar Image", _argAvatar.ViewModel.AvatarImage, typeof(Sprite), false);
            _argAvatar.ViewModel.AppImage = (Sprite)EditorGUILayout.ObjectField("App Image", _argAvatar.ViewModel.AppImage, typeof(Sprite), false);

            if (_argAvatar.ViewModel.Style.Size == AvatarSize.Large)
            {
                _argAvatar.ViewModel.IndicatorColor = EditorGUILayout.ColorField("Indicator Color", _argAvatar.ViewModel.IndicatorColor);
                _argAvatar.ViewModel.CounterText = EditorGUILayout.TextField("Indicator Text", _argAvatar.ViewModel.CounterText);
                _argAvatar.ViewModel.ShowLabelOnHover = EditorGUILayout.Toggle("Show Label On Hover", _argAvatar.ViewModel.ShowLabelOnHover);

                if (_argAvatar.ViewModel.ShowLabelOnHover)
                {
                    _argAvatar.ViewModel.LabelText = EditorGUILayout.TextField("Label Text", _argAvatar.ViewModel.LabelText);
                }
            }

            serializedObject.ApplyModifiedProperties();
        }

        private void SwitchPrefab()
        {
            if (EditorUtilities.IsInSceneAndNotInPrefabEditMode(_argAvatar.gameObject))
            {
                // target is in the scene (not persistent) and not in prefab edit mode, execute the code
                _argAvatar.PopulatePrefab();
            }
        }
    }
}
#endif
