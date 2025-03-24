using System;
using Oculus.Interaction;
using UIComponents.Runtime;
using UnityEngine;

namespace View.UI
{
    public class ManualInstructionPill : SimpleAnimatedCanvasGroup
    {
        [SerializeField] private TMPro.TextMeshProUGUI _text;
        [SerializeField] private PokeInteractable _pokeInteractable;
        public Action<string> OnInstructionSent;
        
        public void SetText(string text)
        {
            _text.text = text;
        }

        public void SendInstruction()
        {
            var instruction = _text.text;
            
            OnInstructionSent?.Invoke(instruction);
            Debug.Log("Instruction Sent: " + instruction);
        }
    }
}