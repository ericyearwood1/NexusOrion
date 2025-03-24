using ARGlasses.Interaction;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using UnityEngine;

namespace ARGlasses.Components
{
    public class ArgTooltip : ViewController<TooltipViewModel, ButtonModel, TooltipView>
    {

        private OCInsetAttachData _attachData;
        
        private void Start()
        {
            InitializeComponents(ensure: true);
            
            View.Initialize(InteractionModel);

            this.Ensure(ref _attachData);
            SetOffset(ViewModel.FromDirection);

            ViewModel.SourceTarget.WhenStateChanged += OnSourceTargetStateChanged;
            InteractionModel.WhenStateChanged += HandleTooltipStateChanged;
            
            View.OnHidden.AddListener(()=>Destroy(gameObject));
            View.Show();
        }

        private void OnSourceTargetStateChanged(TargetState state)
        {
            switch (state)
            {
                case TargetState.Normal:

                    View.Hide(ViewModel.DespawnDelay);

                    break;
                case TargetState.Hover:

                    View.Show();

                    break;
                case TargetState.Press:
                    break;
            }
        }

        private void HandleTooltipStateChanged(TargetState tooltipState)
        {
            switch (tooltipState)
            {
                case TargetState.Normal:
                    
                    View.Hide(ViewModel.DespawnDelay);
                    
                    break;
                case TargetState.Hover:
                    
                    View.Show();
                    break;
            }
        }

        private void SetOffset(FromDirection direction)
        {
            switch (direction)
            {
                case FromDirection.Bottom:
                    // Position below the element
                    _attachData.SetYOffset(-ViewModel.Offset);
                    _attachData.SetYPivot(1f);
                    _attachData.SetYAlignment(OCLayoutPadding.Alignment.NegOut);
                    
                    break;
                case FromDirection.Top:
                    // Position above the element
                    _attachData.SetYOffset(ViewModel.Offset);
                    _attachData.SetYPivot(0f);
                    _attachData.SetYAlignment(OCLayoutPadding.Alignment.PosOut);
                    
                    break;
                case FromDirection.Left:
                    // Position to the left of the element
                    _attachData.SetXOffset(-ViewModel.Offset);
                    _attachData.SetXPivot(1f);
                    _attachData.SetXAlignment(OCLayoutPadding.Alignment.NegOut);
                    
                    break;
                case FromDirection.Right:
                    // Position to the right of the element
                    _attachData.SetXOffset(ViewModel.Offset);
                    _attachData.SetXPivot(0f);
                    _attachData.SetXAlignment(OCLayoutPadding.Alignment.PosOut);
                    
                    break;
            }
            
            OCLayoutSingletonDriver.ForceUpdateNowForAllLayoutInScene();
        }

        private void OnDestroy()
        {
            ViewModel.SourceTarget.WhenStateChanged -= OnSourceTargetStateChanged;
            InteractionModel.WhenStateChanged -= HandleTooltipStateChanged;
        }
    }
}