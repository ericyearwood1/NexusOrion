using System;
using System.Collections.Generic;
using UIComponents.Runtime;
using UnityEngine;
using View.UI;

namespace View
{
    public class DefaultInstructionsPanel : SimpleAnimatedCanvasGroup
    {
        private const int INSTRUCTION_POOL_SIZE = 20;
        public Action<string> OnInstructionSent;
        [SerializeField] private ManualInstructionPill _instructionPrefab;
        [SerializeField] private Transform _instructionContainer;
        
        private List<ManualInstructionPill> _instructionPool = new();
        private int _instructionCount;
        
        private void Awake()
        {
            for (var i = 0; i < INSTRUCTION_POOL_SIZE; i++)
            {
                var instruction = Instantiate(_instructionPrefab, _instructionContainer);
                instruction.gameObject.SetActive(false);
                _instructionPool.Add(instruction);
                instruction.OnInstructionSent += InstructionSentHandler;
            }
        }

        private void InstructionSentHandler(string instruction)
        {
            Debug.Log($"HandUITesting::DefaultInstructionsPanel:InstructionSentHandler {instruction}");
            OnInstructionSent?.Invoke(instruction);
        }

        public void SetDefaultInstructions(List<string> instructions)
        {
            Debug.Log($"Set Default Instructions {instructions.Count}");
            _instructionCount = instructions.Count;
            for (var i = 0; i < _instructionCount; i++)
            {
                if (i >= _instructionPool.Count)
                {
                    var instruction = Instantiate(_instructionPrefab, _instructionContainer);
                    _instructionPool.Add(instruction);
                }

                var instructionText = _instructionPool[i];
                instructionText.gameObject.SetActive(true);
                instructionText.SetText(instructions[i]);
            }
            
            for (var i = instructions.Count; i < _instructionPool.Count; i++)
            {
                _instructionPool[i].gameObject.SetActive(false);
            }
        }

        // public new void Hide()
        // {
        //     base.Hide();
        //     // foreach (var instruction in _instructionPool)
        //     // {
        //     //     instruction.gameObject.SetActive(false);
        //     // }
        // }

        // public new void Show()
        // {
        //     base.Show();
        //     // for (var i = 0; i < _instructionCount; i++)
        //     // {
        //     //     _instructionPool[i].gameObject.SetActive(true);
        //     // }
        // }
    }
}