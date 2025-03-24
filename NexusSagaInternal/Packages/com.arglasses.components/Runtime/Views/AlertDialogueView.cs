using System;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;

namespace ARGlasses.Components
{
    public class AlertDialogueView : MonoBehaviour, IArgView<AlertDialogueViewModel, ViewController>
    {
        [SerializeField] private AlertDialogueStyle _style;
        public AlertDialogueStyle Style => _style;

        [SerializeField] private List<ArgButton> _roundButtons;
        [SerializeField] private List<ArgButton> _textButtons;
        [SerializeField] private LabelProvider _title;
        [SerializeField] private LabelProvider _description;
        [SerializeField] private BackgroundProvider _bg;
        [SerializeField] private BackgroundProvider _appImage;

        [SerializeField] private Color _fromSystemBgColor;
        [SerializeField] private Color _fromSystemBorderColor;
        [SerializeField] private Color _fromAugmentBgColor;
        [SerializeField] private Color _fromAugmentBorderColor;



        public void UpdateViewData(AlertDialogueViewModel viewModel)
        {
            _title.SetLabel(viewModel.Title);
            _description.SetLabel(viewModel.Desc);
            _textButtons[0].ViewModel.LabelText = viewModel.ButtonText1;
            _textButtons[1].ViewModel.LabelText = viewModel.ButtonText2;
            _textButtons[2].ViewModel.LabelText = viewModel.ButtonText3;
            _roundButtons[0].ViewModel.Icon = viewModel.ButtonIcon1;
            _roundButtons[1].ViewModel.Icon = viewModel.ButtonIcon2;
            _roundButtons[2].ViewModel.Icon = viewModel.ButtonIcon3;
            _appImage.BackgroundRenderer.Sprite = viewModel.AppImage;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
            EditorUtility.SetDirty(_title);
            EditorUtility.SetDirty(_description);
#endif
        }

        public void UpdateViewStyle(AlertDialogueStyle style)
        {
            _style = style;

            switch (_style.Source)
            {
                case AlertDialogueSource.Augment:
                    _bg.BackgroundRenderer.SetColorA(_fromAugmentBgColor);
                    _bg.BackgroundRenderer.SetBorderColor(_fromAugmentBorderColor);
                    break;
                case AlertDialogueSource.System:
                    _bg.BackgroundRenderer.SetColorA(_fromSystemBgColor);
                    _bg.BackgroundRenderer.SetBorderColor(_fromSystemBorderColor);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            switch (_style.FromStyle)
            {
                case AlertDialogueFromStyle.FromApp:
                    _appImage.gameObject.SetActive(true);
                    break;
                case AlertDialogueFromStyle.System:
                    _appImage.gameObject.SetActive(false);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            _description.gameObject.SetActive(_style.HasDescription);

            switch (_style.ButtonType)
            {
                case ButtonType.Text:
                    _roundButtons.ForEach(x => x.gameObject.SetActive(false));
                    for (var i = 0; i < _textButtons.Count; i++)
                    {
                        _textButtons[i].gameObject.SetActive(i < _style.NumberOfButtons);
                    }

                    break;
                case ButtonType.Round:
                    _textButtons.ForEach(x => x.gameObject.SetActive(false));
                    for (var i = 0; i < _roundButtons.Count; i++)
                    {
                        _roundButtons[i].gameObject.SetActive(i < _style.NumberOfButtons);
                    }

                    break;
                case ButtonType.App:
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        public void Initialize(ViewController controller)
        {
            var alertDialogueController = controller as ArgAlertDialogue;

            foreach (var roundButton in _roundButtons)
            {
                roundButton.WhenClick.RemoveAllListeners();
            }

            foreach (var textButton in _textButtons)
            {
                textButton.WhenClick.RemoveAllListeners();
            }

            if(alertDialogueController != null)
            {
                _roundButtons[0].WhenClick.AddListener(() => alertDialogueController.Button1Clicked?.Invoke());
                _textButtons[0].WhenClick.AddListener(() => alertDialogueController.Button1Clicked?.Invoke());
                _roundButtons[1].WhenClick.AddListener(() => alertDialogueController.Button2Clicked?.Invoke());
                _textButtons[1].WhenClick.AddListener(() => alertDialogueController.Button2Clicked?.Invoke());
                _roundButtons[2].WhenClick.AddListener(() => alertDialogueController.Button3Clicked?.Invoke());
                _textButtons[2].WhenClick.AddListener(() => alertDialogueController.Button3Clicked?.Invoke());
            }
        }

        private void OnDestroy()
        {
            foreach (var roundButton in _roundButtons)
            {
                roundButton.WhenClick.RemoveAllListeners();
            }

            foreach (var textButton in _textButtons)
            {
                textButton.WhenClick.RemoveAllListeners();
            }
        }


        public void Select(bool on)
        {
            throw new NotImplementedException();
        }
    }
}
