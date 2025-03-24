#if UNITY_EDITOR
using UnityEditor;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(RadialProgressBar))]
    public class RadialProgressBarControllerEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            // Get the target script
            RadialProgressBar progressBarController = (RadialProgressBar)target;

            // Draw a slider for the fill percentage
            progressBarController.FillPercentage =
                EditorGUILayout.Slider("Fill Percentage", progressBarController.FillPercentage * 100, 0, 100) * 0.01f;
            progressBarController.StartPercentage =
                EditorGUILayout.Slider("Start Percentage", progressBarController.StartPercentage * 100, 0, 100) * 0.01f;
            progressBarController.RotationDegPerSecond = EditorGUILayout.FloatField("Rotation Deg Per Second",
                progressBarController.RotationDegPerSecond);
        }
    }
}
#endif
