
using UnityEditor;

namespace ARGlasses.Interaction.Motion
{

    [InitializeOnLoad]
    public class PlayModeStateListener
    {
        static PlayModeStateListener()
        {
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        }

        private static void OnPlayModeStateChanged(PlayModeStateChange playModeState)
        {
            if (playModeState == PlayModeStateChange.ExitingPlayMode)
            {
                MotionUtils.ResetRunner();
            }
        }
    }
}
