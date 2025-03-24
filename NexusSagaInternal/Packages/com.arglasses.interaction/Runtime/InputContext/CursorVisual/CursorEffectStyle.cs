using UnityEngine;

namespace ARGlasses.Interaction
{
    public class CursorEffectStyle : MonoBehaviour
    {
        // place this on a game object to make every Target
        // below it in the hierarchy ignore the cursor hover effect
        public enum Mode
        {
            None = -1,
            ShowPointer = 0,
            HideGlow = 1
        }
        [SerializeField] private Mode _mode = Mode.ShowPointer;
        public Mode SelectedMode => _mode;
    }
}
