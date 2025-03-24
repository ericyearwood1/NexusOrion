using Multiplayer.Runtime.Services;
using UnityEngine;
using View;

namespace Systems
{
    public class ManualInstructionPanelVisualisationSystem : IMultiplayerFeatureService
    {
        private readonly RobotHomeUI _robotHomeUI;
        private readonly RecordInstructionPanel _recordPanel;

        public ManualInstructionPanelVisualisationSystem(RobotHomeUI robotHomeUI, RecordInstructionPanel recordPanel)
        {
            _robotHomeUI = robotHomeUI;
            _recordPanel = recordPanel;
            _robotHomeUI.HideDefaultInstructions();
        }
        
        public void Activate()
        {
            Debug.Log("ManualInstructionPanelVisualisationSystem::Activate");
            _robotHomeUI.ShowDefaultInstructions();
            _recordPanel.ShowDefaultInstructions();
        }

        public void Deactivate()
        {
            Debug.Log("ManualInstructionPanelVisualisationSystem::Deactivate");
            _robotHomeUI.HideDefaultInstructions();
            _recordPanel.HideDefaultInstructions();
        }
    }
}