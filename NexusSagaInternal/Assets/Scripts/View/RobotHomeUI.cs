using System.Collections.Generic;
using UnityEngine;

namespace View
{
    public class RobotHomeUI : MonoBehaviour
    {
        public DefaultInstructionsPanel defaultInstructionsPanel;
        private bool _isShow;

        public void SetDefaultInstructions(List<string> list)
        {
            defaultInstructionsPanel.SetDefaultInstructions(list);
        }
        
        public void ShowDefaultInstructions()
        {
            Debug.Log("RobotHomeUI:ShowDefaultInstructions");
            defaultInstructionsPanel.Show();
        }
        
        public void HideDefaultInstructions()
        {
            Debug.Log("RobotHomeUI:HideDefaultInstructions");
            defaultInstructionsPanel.Hide();
        }
    }
}