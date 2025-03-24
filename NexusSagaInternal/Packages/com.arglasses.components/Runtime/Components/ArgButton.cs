using System;
using System.ComponentModel;
using ARGlasses.Interaction;
using ITK;
using UnityEngine;
using UnityEngine.Events;
using Component = UnityEngine.Component;

namespace ARGlasses.Components
{
    public class ArgButton : ViewController<ButtonViewModel, ButtonModel, ButtonView>, ITargetSizeProvider
    {
        [Group("Events")] public UnityEvent WhenClick, WhenHoverEnter, WhenHoverExit;
        [Group("Events")] public UnityEvent<bool> WhenValueChanged;

        private TargetState _prevTargetState = TargetState.Normal;

        public bool Interactable
        {
            get => _interactable;
            set => SetInteractive(value);
        } 
        [SerializeField] private bool _interactable = true;
        
        private void Start()
        {
            InitializeComponents(ensure: true);
            ForceUpdateView();
            View.Initialize(InteractionModel);
            if (View.Toggleable)
            {
                var registerToggleEvent = new SegmentedToggleEvent(this, ViewModel.Selected, this);
                EventBubbler.Bubble<ISegmentedToggleEventHandler, SegmentedToggleEvent>(registerToggleEvent,
                    (handler, @event) => { handler.RegisterToggle(registerToggleEvent); });
            }
            
            InteractionModel.WhenClicked += Clicked;
            InteractionModel.WhenStateChanged += StateChanged;

            SetInteractive(Interactable);
            
            ViewModel.PropertyChanged += ViewModelOnPropertyChanged;
        }

        private void ViewModelOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == "Selected")
            {
                WhenValueChanged?.Invoke(ViewModel.Selected);
            }
        }

        private void Clicked()
        {
            if (!Interactable) return;
            
            if (View.Toggleable)
            {
                var toggleEvent = new SegmentedToggleEvent(this, ViewModel.Selected, this);
                EventBubbler.Bubble<ISegmentedToggleEventHandler, SegmentedToggleEvent>(toggleEvent,
                    (handler, eventArgs) => { handler.ApplyToggle(eventArgs); });

                if (toggleEvent.HandlePolicy is BubbleEventHandlePolicy.None)
                {
                    //no parent handler found. change selected on self
                    ViewModel.Selected = !ViewModel.Selected;
                    View.Select(ViewModel.Selected);
                    
                }
            }

            WhenClick?.Invoke();
        }

        private void OnDestroy()
        {
            if (InteractionModel != null)
            {
                InteractionModel.WhenClicked -= Clicked;
                InteractionModel.WhenStateChanged -= StateChanged;
            }
        }

        private void StateChanged(TargetState state)
        {
            switch (state)
            {
                case TargetState.None:
                    if (_prevTargetState.IsHoverOrPress()) WhenHoverExit?.Invoke();
                    break;
                case TargetState.Normal:
                    if (_prevTargetState.IsHoverOrPress()) WhenHoverExit?.Invoke();
                    break;
                case TargetState.Hover:
                    if (_prevTargetState.IsNormal()) WhenHoverEnter?.Invoke();
                    break;
                case TargetState.Press:
                    if (_prevTargetState.IsNormal() || _prevTargetState.IsDrift()) WhenHoverEnter?.Invoke();
                    break;
                case TargetState.Drift:
                    if (_prevTargetState.IsPress()) WhenHoverExit.Invoke();
                    break;
                case TargetState.Disabled:
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(state), state, null);
            }

            _prevTargetState = state;
        }

        private void SetInteractive(bool value)
        {
            if (InteractionModel == null || InteractionModel.Target == null) return;

            InteractionModel.Target.Mute = value ? TargetState.None : TargetState.Any;
        }
        
        protected void OnValidate()
        {
            ForceUpdateView();
        }

        public Component Component => this;
        public Vector3 GetOverrideSize(Target target)
        {
            if(View.BackgroundProvider) return View.BackgroundProvider.CurrentSize;
            return GetComponent<RectTransform>().rect.size;
        }
    }
}
