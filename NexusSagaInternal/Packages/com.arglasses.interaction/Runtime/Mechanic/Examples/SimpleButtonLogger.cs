using UnityEngine;

namespace ARGlasses.Interaction
{
    public class SimpleButtonLogger : MonoBehaviour
    {
        [SerializeField] private ButtonModel _buttonModel;

        protected void Awake()
        {
            this.Ensure(ref _buttonModel);
            _buttonModel.WhenClicked += HandleClicked;
        }

        private void HandleClicked()
        {
            Debug.Log("Clicked");
        }
    }
}
