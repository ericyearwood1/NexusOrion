using TMPro;
using UnityEngine;

namespace Robot.Runtime.View
{
    public class LabelHighlightView : HighlightView
    {
        [SerializeField] protected TMP_Text _label;
        
        public virtual void Show(Transform cameraTransform, string label)
        {
            _label.text = label.Replace("_"," ");
            StartShow(cameraTransform);
        }

        protected override void UpdateDisplay(float progress)
        {
            base.UpdateDisplay(progress);
            _label.alpha = progress;
        }
    }
}