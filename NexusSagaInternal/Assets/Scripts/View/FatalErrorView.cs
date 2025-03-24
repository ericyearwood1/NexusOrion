using TMPro;
using UnityEngine;

namespace View
{
    public class FatalErrorView : SiroUIView
    {
        [SerializeField] private TMP_Text _errorText;
        public void Initialise(string error)
        {
            _errorText.text = error;
        }
    }
}