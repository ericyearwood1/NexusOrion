using TMPro;
using UnityEngine;

namespace UIComponents.Runtime
{
    public class GeneralMessagePanel : SimpleAnimatedCanvasGroup
    {
        [SerializeField] private TMP_Text _message;

        public void SetMessage(string message)
        {
            _message.text = message;
            Show();
        }
    }
}