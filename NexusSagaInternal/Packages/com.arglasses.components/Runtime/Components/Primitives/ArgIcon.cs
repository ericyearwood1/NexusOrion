using System;
using System.Text.RegularExpressions;
using OSIG.Tools.Layout;
using OSIG.Tools.StateMachines;
using ProtoKit.UI;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    /// <summary>
    /// tells controller where to assign icons
    /// </summary>
    public class ArgIcon : MonoBehaviour
    {
        [SerializeField]
        [RequireStates(PKUIUtils.DEFAULT_NORMAL, PKUIUtils.DEFAULT_HOVERED, PKUIUtils.DEFAULT_PRESSED,
            PKUIUtils.DEFAULT_SELECTED)]
        private StateMachine _motionMachine = new StateMachine();

        [SerializeField] private StateMapping _stateMapping = new StateMapping()
        {
            Normal = PKUIUtils.DEFAULT_NORMAL,
            Hovered = PKUIUtils.DEFAULT_HOVERED,
            Pressed = PKUIUtils.DEFAULT_PRESSED,
            Selected = PKUIUtils.DEFAULT_SELECTED
        };

        public StateMachine MotionMachine => _motionMachine;

        public PKUIPanel IconRenderer;
        [FormerlySerializedAs("LayoutSize")] public OCLayoutSize IconLayoutSize;
        public OCInsetAttachData AttachData;

        public void SetIcon(Sprite icon)
        {
            if (IconRenderer != null) IconRenderer.Sprite = icon;
            else Debug.LogWarning("no icon renderer on argIcon");
        }

        public void TransitionToState(string state)
        {
            if(_motionMachine.IsCreated) _motionMachine.TransitionToState(RemoveOnOff(state));
        }

        private string RemoveOnOff(string input)
        {
            // The pattern looks for strings that end with either _On or _Off
            string pattern = "(_On|_Off)$";
            return Regex.Replace(input, pattern, "");
        }
        private void Awake()
        {
            if (_motionMachine.Definition == null) return;

            _motionMachine.Create(this, _stateMapping.Normal);
          //  _motionMachine.Bind<Vector2>("IconSize", size => { if (IconLayoutSize != null) IconLayoutSize.SetSize2D(size); });
            _motionMachine.Bind<float>("IconLiftZ", liftZ => { if (AttachData != null) AttachData.SetPaddingBack(liftZ); });
        }
    }
}
