using System;
using ARGlasses.Interaction;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using OSIG.Tools.StateMachines;
using OSIG.Tools.Units;
using UnityEngine;

namespace ARGlasses.Components
{
    public class ArgOverflowMenu : MonoBehaviour
    {
        [SerializeField] private ArgButton _visibilityToggle;
        
        //[SerializeField] private LabelProvider _visibilityButtonLabel; 
        
        [Space(10)] 
        [SerializeField] private CanvasGroupFadeView _canvasFadeView;

        [SerializeField] private OCLayoutSize _contentLayoutSize;

        [SerializeField] private OCLayoutStack _contentLayout;

        [SerializeField] private OCInsetAttachData _contentParentAttachData;
        [SerializeField] private OCSizeAttachData _contentAttachData;
        
        [Space(10)]
        [SerializeField] private OCLayoutStack.StackDirection _direction;
        [SerializeField] private bool _showVisibilityButtonLabel; // Used in editor script
        
        [Space(10)]
        [SerializeField] private StateMachine _stateMachine;
        
        private bool _debugging = false;
        private bool _expanded = false;

        private float _panelMinimizedSize = 100f;
        private float _panelExpandedSize;

        public Action OnExpanded;
        
        private void Start()
        {
            InitializeLayoutValues();
            
            if (_debugging)
            {
                Debug.Log($"Content Panel current size: {_contentLayoutSize.CurrentSize} // Conversion ratio: {_contentLayoutSize.UnitsContext.GetConversionRatio(OCUnits.Pixels)}");
                Debug.Log($"Panel expanded size: {_panelExpandedSize}");
            }
            
            if(_visibilityToggle) _visibilityToggle.WhenValueChanged.AddListener(SetExpanded);
            
            _stateMachine.Create(this, "Minimized");
            _stateMachine.OnBeginTransitioningIn += HandleStateChange;
        }

        private void InitializeLayoutValues()
        {
            OCLayoutSingletonDriver.ForceUpdateNowForAllLayoutInScene();
            var contentSize = _contentLayoutSize.CurrentSize / _contentLayoutSize.UnitsContext.GetConversionRatio(OCUnits.Pixels);

            switch (_direction)
            {
                case OCLayoutStack.StackDirection.LeftToRight:
                case OCLayoutStack.StackDirection.RightToLeft:
                    
                    _panelExpandedSize = contentSize.x;
                    
                    break;
                case OCLayoutStack.StackDirection.TopToBottom:
                case OCLayoutStack.StackDirection.BottomToTop:
                    
                    _panelExpandedSize = contentSize.y;
                    
                    break;
                case OCLayoutStack.StackDirection.BackToFront:
                case OCLayoutStack.StackDirection.FrontToBack:
                    
                    _panelExpandedSize = contentSize.z;
                    break;
            }
        }

        private void HandleStateChange(StateMachine.StateChangeEventArgs value)
        {
            switch (value.State)
            {
                case "Minimized":
                    _canvasFadeView.SetVisibility(false);
                    break;
                case "Expanded":
                    _canvasFadeView.SetVisibility(true);
                    break;
            }
        }

        public void HandleTransitionProgress(float i)
        {
            switch (_direction)
            {
                case OCLayoutStack.StackDirection.LeftToRight:
                case OCLayoutStack.StackDirection.RightToLeft:
                    
                    var width = Mathf.Lerp(_panelMinimizedSize, _panelExpandedSize, i);
                    _contentLayoutSize.SetWidth(width);
                    
                    break;
                case OCLayoutStack.StackDirection.TopToBottom:
                case OCLayoutStack.StackDirection.BottomToTop:
                    
                    var height = Mathf.Lerp(_panelMinimizedSize, _panelExpandedSize, i);
                    _contentLayoutSize.SetHeight(height);
                    
                    break;
                case OCLayoutStack.StackDirection.BackToFront:
                case OCLayoutStack.StackDirection.FrontToBack:
                    
                    var depth = Mathf.Lerp(_panelMinimizedSize, _panelExpandedSize, i);
                    _contentLayoutSize.SetDepth(depth);
                    
                    break;
            }
        }
        
        public void SetExpanded(bool value)
        {
            _expanded = value;
            
            if(_expanded)
                OnExpanded?.Invoke();
            
            if (_stateMachine.IsCreated)
            {
                var state = _expanded ? "Expanded" : "Minimized";
                _stateMachine.TransitionToState(state);
                
                if (_debugging) Debug.Log($"Overflow Menu state: <b>{state}</b>");
            }
            else
            {
                if (_debugging) Debug.Log($"Overflow Menu state machine not created.");
            }
        }

        public bool IsExpanded()
        {
            return _expanded;
        }

        public void UpdateLayout()
        {
            if (_contentParentAttachData == null || _contentAttachData == null)
            {
                Debug.LogWarning("Overflow Menu is missing attach data references.", gameObject);
                return;
            }
            
            _contentLayout.Direction = _direction;
            
            _contentParentAttachData.Set3DAlignment(OCLayoutPadding.Alignment.Center);
            _contentAttachData.Set3DAlignment(OCLayoutPadding.Alignment.Center);
            
            switch (_direction)
            {
                case OCLayoutStack.StackDirection.LeftToRight:
                    _contentParentAttachData.SetXAlignment(OCLayoutPadding.Alignment.PosOut);
                    _contentAttachData.SetXAlignment(OCLayoutPadding.Alignment.NegIn);
                    break;
                case OCLayoutStack.StackDirection.RightToLeft:
                    _contentParentAttachData.SetXAlignment(OCLayoutPadding.Alignment.NegOut);
                    _contentAttachData.SetXAlignment(OCLayoutPadding.Alignment.PosIn);
                    break;
                case OCLayoutStack.StackDirection.TopToBottom:
                    _contentParentAttachData.SetYAlignment(OCLayoutPadding.Alignment.NegOut);
                    _contentAttachData.SetYAlignment(OCLayoutPadding.Alignment.PosIn);
                    break;
                case OCLayoutStack.StackDirection.BottomToTop:
                    _contentParentAttachData.SetYAlignment(OCLayoutPadding.Alignment.PosOut);
                    _contentAttachData.SetYAlignment(OCLayoutPadding.Alignment.NegIn);
                    break;
                case OCLayoutStack.StackDirection.BackToFront:
                    _contentParentAttachData.SetZAlignment(OCLayoutPadding.Alignment.NegOut);
                    _contentAttachData.SetZAlignment(OCLayoutPadding.Alignment.PosIn);
                    break;
                case OCLayoutStack.StackDirection.FrontToBack:
                    _contentParentAttachData.SetZAlignment(OCLayoutPadding.Alignment.PosOut);
                    _contentAttachData.SetZAlignment(OCLayoutPadding.Alignment.NegIn);
                    break;
            }

        }

        public void SetLabelVisibility(bool visible)
        {
            if (_visibilityToggle == null)
            {
                Debug.LogWarning("Visibility Toggle is not assigned.", gameObject);
                return;
            }
            
            _showVisibilityButtonLabel = visible;

            var styleEnforcer = _visibilityToggle.GetComponentInChildren<ButtonStyleEnforcer>();

            if (styleEnforcer == null)
            {
                Debug.LogWarning($"ButtonStyleEnforcer is missing, unable to set label visibility on {gameObject.name}", gameObject);
            }
            
            var buttonStyle = styleEnforcer.Style;
            
            buttonStyle.HideLabel = visible; // Need to invert this due to HideLabel having inconsistent behavior between ButtonTypes
            buttonStyle.ButtonType = visible ? ButtonType.Text : ButtonType.Round;
            
            styleEnforcer.SetStyleAndPopulate(buttonStyle);
        }
    }
    
}