using OSIG.Tools.StateMachines;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace ARGlasses.Components
{
    public class ViewStateVisualsOverride : MonoBehaviour
    {
        public StateMachine ColorMachineOverride;
        public StateMachine MotionMachineOverride;

        public StateMachineDefinition CursorStateMachineDefinition;
        private void Awake()
        {
            var viewStateVisuals = GetComponentInChildren<ViewStateVisuals>();

            if (viewStateVisuals == null)
                return;

            viewStateVisuals.InjectOverrides(this);
        }

        public void ForceUpdate()
        {
            OnValidate();
        }

        private void OnValidate()
        {
            var viewStateVisuals = GetComponentInChildren<ViewStateVisuals>();

            if (viewStateVisuals == null)
                return;

            viewStateVisuals.InjectOverrides(this);

#if UNITY_EDITOR

            if (!EditorApplication.isPlayingOrWillChangePlaymode && !EditorApplication.isPlaying)
            {
                var viewController = gameObject.GetComponent<ViewController>();
                if (viewController != null)
                {
                    viewController.ForceUpdateViewNoModel();
                }
            }
#endif
        }
    }
}
