using System;
using System.Linq;
using ARGlasses.Interaction;
using OSIG.Tools.StateMachines;
using UnityEngine;

namespace ARGlasses.Components
{
    /// <summary>
    /// Base class for handling the visual states of AR components.
    /// </summary>
    public abstract class ViewStateVisuals : MonoBehaviour
    {
        protected ViewStateVisualsOverride _viewStateVisualsOverride;
        public abstract StateMachine ColorMachine { get; }
        public abstract StateMachine MotionMachine { get; }

        public abstract void InitializeStateMachine();
        public abstract void DestroyStateMachine();

        public void InjectOverrides(ViewStateVisualsOverride viewStateVisualsOverride)
        {
            _viewStateVisualsOverride = viewStateVisualsOverride;
        }

        public bool HasColorMachineOverride()
        {
            return _viewStateVisualsOverride != null && _viewStateVisualsOverride.ColorMachineOverride != null &&
                   _viewStateVisualsOverride.ColorMachineOverride.Definition != null;
        }

        public bool HasMotionMachineOverride()
        {
            return _viewStateVisualsOverride != null && _viewStateVisualsOverride.MotionMachineOverride != null &&
                   _viewStateVisualsOverride.MotionMachineOverride.Definition != null;
        }

        public virtual void Select(bool on)
        {
        }

        public bool TryExtractComponentData<TComponent, TComponentData>(StateMachine stateMachine, string variableName,
            string stateName, out TComponentData data) where TComponent : BasicVariableComponent<TComponentData>
        {
            var component = stateMachine.Definition.Components.OfType<TComponent>()
                .FirstOrDefault(c => c.VariableName == variableName);

            if (component != null)
            {
                var stateIndex = FindStateIndex(stateMachine, stateName);
                if (stateIndex != -1)
                {
                    data = component.PerStateData[stateIndex];
                    return true;
                }
            }

            data = default;
            return false;
        }

        private int FindStateIndex(StateMachine stateMachine, string stateName)
        {
            for (var i = 0; i < stateMachine.Definition.States.Count; i++)
            {
                if (stateMachine.Definition.States[i].Name != stateName)
                {
                    continue;
                }

                return i;
            }

            return -1;
        }
    }

    /// <summary>
    /// Generic version of ViewStateVisuals for type-safe handling of model and view.
    /// </summary>
    public abstract class ViewStateVisuals<Tmodel, Tview> : ViewStateVisuals
        where Tmodel : class
        where Tview : class
    {
        protected Tmodel _model;
        protected Tview _view;

        /// <summary>
        /// Sets the model and view, and subscribes/unsubscribes to their respective events.
        /// </summary>
        public virtual void SetModelAndView(Tmodel model, Tview view)
        {
            if (_model != null)
            {
                UnsubscribeFromModelEvents(_model);
            }


            if (_view != null)
            {
                UnsubscribeFromViewEvents();
            }

            _model = model;
            _view = view;

            if (_model != null)
            {
                SubscribeToModelEvents(_model);
            }

            if (_view != null)
            {
                SubscribeToViewEvents();
            }
        }

        /// <summary>
        /// Subscribes to the events fired by the model.
        /// </summary>
        private void SubscribeToModelEvents(Tmodel model)
        {
            switch (model)
            {
                case ISlider slider:
                    slider.WhenStateChanged += HandleStateChanged;
                    slider.WhenValueNormalizedChanged += HandleValueNormalizedChanged;
                    break;
                case IToggle toggle:
                    toggle.WhenValueChanged += HandleToggleValueChanged;
                    toggle.WhenStateChanged += HandleStateChanged;
                    if (toggle is ToggleSliderModel toggleSliderModel)
                        toggleSliderModel.SliderModel.Drift.WhenDragStateChanged += HandleDragStateChanged;
                    break;
                case IButton button:
                    button.WhenClicked += HandleClicked;
                    button.WhenCancelled += HandleCancelled;
                    button.WhenStateChanged += HandleStateChanged;
                    break;
            }
        }

        private void UnsubscribeFromModelEvents(Tmodel model)
        {
            switch (model)
            {
                case ISlider slider:
                    slider.WhenStateChanged -= HandleStateChanged;
                    slider.WhenValueNormalizedChanged -= HandleValueNormalizedChanged;
                    break;
                case IToggle toggle:
                    toggle.WhenValueChanged -= HandleToggleValueChanged;
                    toggle.WhenStateChanged -= HandleStateChanged;
                    if (toggle is ToggleSliderModel toggleSliderModel)
                        toggleSliderModel.SliderModel.Drift.WhenDragStateChanged -= HandleDragStateChanged;
                    break;
                case IButton button:
                    button.WhenClicked -= HandleClicked;
                    button.WhenCancelled -= HandleCancelled;
                    button.WhenStateChanged -= HandleStateChanged;
                    break;
            }
        }

        protected virtual void UnsubscribeFromViewEvents()
        {
            switch (_view)
            {
                case TileButtonView tbv:
                    tbv.SelectChanged -= HandleToggleValueChanged;
                    break;
                case ButtonView buttonView:
                    buttonView.SelectChanged -= HandleToggleValueChanged;
                    break;
                case ListItemView listItemView:
                    listItemView.SelectChanged -= HandleToggleValueChanged;
                    break;
            }
        }

        protected virtual void SubscribeToViewEvents()
        {
            switch (_view)
            {
                case TileButtonView tbv:
                    tbv.SelectChanged += HandleToggleValueChanged;
                    break;
                case ButtonView buttonView:
                    buttonView.SelectChanged += HandleToggleValueChanged;
                    break;
                case ListItemView listItemView:
                    listItemView.SelectChanged += HandleToggleValueChanged;
                    break;
            }
        }

        protected virtual void HandleToggleValueChanged(bool value)
        {
        }

        protected virtual void HandleStateChanged(TargetState targetState)
        {
        }

        protected virtual void HandleClicked()
        {
        }

        protected virtual void HandleCancelled()
        {
        }

        protected virtual void HandleValueNormalizedChanged(float f)
        {
        }

        protected virtual void HandleDragStateChanged(DragCancelState dragCancelState)
        {
        }


        public abstract void UpdateViewData(Tview view);

        private void OnDestroy()
        {
            SetModelAndView(null, null);
            DestroyStateMachine();
        }
    }
}
