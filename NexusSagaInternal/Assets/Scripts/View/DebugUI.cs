using TMPro;
using UnityEngine;

namespace View
{
    public class DebugUI : MonoBehaviour
    {
        [SerializeField] private TMP_Text _debugText;
        private double _hideTime;
        
        
        public void SetDebugText(string text, float displayTime = 2f)
        {
            _debugText.text = text;
            _hideTime = Time.time + displayTime;
        }
        
        private void Update()
        {
            if (Time.time > _hideTime)
            {
                _debugText.text = "";
            }
        }


        public void Show(bool contextUseDebugUI)
        {
            _debugText.gameObject.SetActive(contextUseDebugUI);
        }
    }
}