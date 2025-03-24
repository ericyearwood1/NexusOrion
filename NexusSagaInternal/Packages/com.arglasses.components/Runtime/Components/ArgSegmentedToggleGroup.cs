using System.Collections.Generic;
using ARGlasses.Interaction;
using UnityEngine;

namespace ARGlasses.Components
{
    public class ArgSegmentedToggleGroup : MonoBehaviour, ISegmentedToggleEventHandler
    {
        private List<ArgButton> _toggleButtons = new List<ArgButton>();

        public void RegisterToggle(SegmentedToggleEvent segmentedToggleEvent)
        {
            if (!_toggleButtons.Contains(segmentedToggleEvent.ToggleButton))
                _toggleButtons.Add(segmentedToggleEvent.ToggleButton);
        }

        public void ApplyToggle(SegmentedToggleEvent segmentedToggleEvent)
        {
            segmentedToggleEvent.Handle( BubbleEventHandlePolicy.StopPropagation);
            foreach (var toggleButton in _toggleButtons)
            {
                if (toggleButton != segmentedToggleEvent.ToggleButton)
                {
                    toggleButton.ViewModel.Selected = false;
                }
                else
                {
                    //this ensures we always have one selected.
                    toggleButton.ViewModel.Selected = true;
                }
            }
        }
    }

    public class SegmentedToggleEvent : BubbleEvent
    {
        public ArgButton ToggleButton { get; private set; }
        public bool IsSelected { get; private set; }

        public SegmentedToggleEvent(ArgButton toggleButton, bool isSelected, Component target)
            : base(target)
        {
            ToggleButton = toggleButton;
            IsSelected = isSelected;
        }
    }

    public interface ISegmentedToggleEventHandler
    {
        void RegisterToggle(SegmentedToggleEvent segmentedToggleEvent);
        void ApplyToggle(SegmentedToggleEvent segmentedToggleEvent);
    }
}
