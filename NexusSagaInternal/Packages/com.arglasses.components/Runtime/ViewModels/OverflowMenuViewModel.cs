using System.Collections;
using System.Collections.Generic;
using ARGlasses.Interaction.Motion;
using UnityEngine;

namespace ARGlasses.Components
{
    public class OverflowMenuViewModel : ViewModelBase
    {
        [Space(10)]
        [SerializeField] private MotionParamsEasing _fadeIn = new(Easing.OutCubic, 0.5f);
        //[SerializeField] private MotionParams _easeIn = new MotionParams(1f, 0.9f, 1f);

        [Space(10)]
        [SerializeField] private MotionParamsEasing _fadeOut = new(Easing.InCubic, 0.75f);
    }
}
