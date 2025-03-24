using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARGlasses.Interaction;
using OSIG.Tools.Layout; 
#if DOTWEEN_ENABLED
using DG.Tweening;
#endif

namespace ARGlasses.Interaction
{
    public class SimpleDwellLogger : MonoBehaviour
    {
        [SerializeField] private MechanicDwell _mechanic;

        //public ARGlasses.Interaction.Motion.MotionParams _MotionParams = new ARGlasses.Interaction.Motion.MotionParams(1.0f, 0.5f, .1f);
        //private ARGlasses.Interaction.Motion.Motion<Vector3> _motion;
        
       /* [SerializeField]
        private ARGlasses.Interaction.Motion.MotionParams _motionParams;
        private ARGlasses.Interaction.Motion.Motion<Vector3> _motion;*/
        
        public OCLayoutStack _ocLayout; 

       // private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenDwell += HandleDwell;
        private void OnDisable() => _mechanic.WhenDwell -= HandleDwell;

        private void HandleDwell(Mechanic.Dwell.Event e)
        {
            var progress = e.Progress;

            if (e.Phase.IsPreExecute())
            {
                Debug.Log($"Not Dwelling yet {progress}");
            }

            // We held the dwell for long enough to execute
            if (e.Phase.IsExecute())
            {
                Debug.Log($"Execute! {progress}");

#if DOTWEEN_ENABLED
                DOTween.To(() => _ocLayout.ScrollPercent, x=>_ocLayout.ScrollPercent = x, 1, 5)
                    .SetId(GetInstanceID());
#endif
                /*
                 DOTween.To(() => _ocLayout.scrollDistance, x=>_ocLayout._scrollDistance(),170, 1 )
                  .SetId(GetInstanceID());
                 */
                // DOTween.To(() => _protokitUIPanel.ColorA, x => _protokitUIPanel.SetColorA(x), color.Value, _transitionTime)
                //     .SetId(GetInstanceID());
                
                //_ocLayout.SetScrollDistance(170);
                //_motion = transform.MoveOneShot(_ocLayout.SetScrollDistance(170), _motionParams);
            }

            // We are still hanging onto the dwell...
            if (e.Phase.IsPostExecute())
            {
                Debug.Log($"Already Dwelling {progress}");
            }
            
        }
    }
}
