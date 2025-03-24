using ARGlasses.Interaction;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using UnityEngine;

namespace ARGlasses.Components.Scroll
{
    [ExecuteAlways]
    public class PoolingScrollerAttachData : OCInsetAttachData
    {
        private CanvasGroup _canvasGroup;

        public CanvasGroup CanvasGroup
        {
            get
            {
                if (_canvasGroup == null)
                    _canvasGroup = GetComponentInChildren<CanvasGroup>();
                return _canvasGroup;
            }
        }

        private ArgListItem _listItem;

        public ArgListItem ListItem
        {
            get
            {
                if (_listItem == null)
                    _listItem = GetComponent<ArgListItem>();
                return _listItem;
            }
        }

        private GameObject _buttonModelGameObject;

        public GameObject ButtonModelGameObject
        {
            get
            {
                if (_buttonModelGameObject == null)
                {
                    var buttonModel = GetComponentInChildren<ButtonModel>();
                    if (buttonModel != null)
                        _buttonModelGameObject = buttonModel.gameObject;
                }

                return _buttonModelGameObject;
            }
        }

        // private ListItemStyleSwitcher _listItemStyleSwitcher;
        //
        // public ListItemStyleSwitcher ListItemStyleSwitcher
        // {
        //     get
        //     {
        //         if (_listItemStyleSwitcher == null)
        //             _listItemStyleSwitcher = GetComponent<ListItemStyleSwitcher>();
        //         return _listItemStyleSwitcher;
        //     }
        // }

        private int _index = -1;
        private ListItemViewModel _viewModel;
        private float _animationProgress = 1;
        private bool _viewActive = true;

        public bool GetViewActive => _viewActive;

        public void SetViewActive(bool active)
        {
            _viewActive = active;
        }

        public void SetViewModel(ListItemViewModel viewModel, float alpha)
        {
            _viewModel = viewModel;

            _animationProgress = alpha;
        }

        //have to update here because can't update layouts in arrangechildren
        private void LateUpdate()
        {
            CanvasGroup.alpha = _viewActive ? 1 : 0;

            if (ButtonModelGameObject != null)
            {
                if (!_viewActive || _animationProgress > .3f || _animationProgress < -.3f)
                {
                    ButtonModelGameObject.SetActive(false);
                }
                else
                {
                    ButtonModelGameObject.SetActive(true);
                }
            }


            if (Mathf.Approximately(_animationProgress, 1) || Mathf.Approximately(_animationProgress, -1))
            {
                CanvasGroup.alpha = 0;
            }

            if (!_viewActive)
            {
                return;
            }

            if (!Equals(ListItem.ViewModel, _viewModel))
            {
                ListItem.ViewModel = _viewModel;
                ListItem.ForceUpdateView();
                OCLayoutSingletonDriver.ForceUpdateNowForAllLayoutInScene();
            }


            // if (ListItemStyleSwitcher != null && !Equals(ListItemStyleSwitcher.Style, _viewModel.Style))
            // {
            //     //ListItemStyleSwitcher.SetStyleAndPopulate(_viewModel.Style);
            //
            //     OCLayoutSingletonDriver.ForceUpdateNowForAllLayoutInScene();
            // }

            if (ListItem != null && !Equals(ListItem.ViewModel, _viewModel))
            {
                ListItem.ViewModel = _viewModel;
                //ListItem.PopulateDependencies();
            }

            VisibilityState edgeState;
            edgeState = _animationProgress > 0 ? VisibilityState.TopEdge : VisibilityState.BottomEdge;
            ListItem.View.DistanceVisibilityChange(Mathf.Abs(_animationProgress), edgeState);
        }
    }
}
