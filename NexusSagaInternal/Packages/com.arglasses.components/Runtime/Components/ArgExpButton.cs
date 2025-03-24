using System.Collections;
using ARGlasses.Interaction;
using UnityEngine;
using UnityEngine.Events;

namespace ARGlasses.Components
{
    public class ArgExpButton : ViewController<ExpButtonViewModel, ButtonModel, ExpButtonView>
    {
        public UnityEvent WhenClick;

        private bool _selected;
        private Coroutine _deselectTimer;

        private void Start()
        {
            InitializeComponents(ensure:true);
            ForceUpdateView();
            View.Initialize(InteractionModel);
            InteractionModel.WhenClicked += Clicked;
        }

        private void Clicked()
        {
            //todo reset this with a timer, or launcher callback
            if (!_selected)
            {
                Selected = true;
                _deselectTimer = StartCoroutine(DeselectTimer());
            }

            WhenClick?.Invoke();
        }

        private void OnDestroy()
        {
            Selected = false;
            InteractionModel.WhenClicked -= Clicked;
            StopCoroutine(DeselectTimer());
        }


        /// <summary>
        /// removes progress overlay
        /// </summary>
        /// <returns></returns>
        private IEnumerator DeselectTimer()
        {
            yield return new WaitForSeconds(3);
            OnDeselectTimerFinished();
        }

        private void OnDeselectTimerFinished()
        {
            Selected = false;
            _deselectTimer = null;
        }

        private void OnDisable()
        {
            if(_deselectTimer != null)
                StopCoroutine(DeselectTimer());
            Selected = false;
        }

        protected void OnValidate()
        {
            ForceUpdateView();
        }

        public bool Selected
        {
            get => _selected;
            set
            {
                _selected = value;
                View.Select(_selected);
            }
        }

    }
}
