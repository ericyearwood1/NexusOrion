using OSIG.Tools.StateMachines;
using UnityEngine;

namespace ARGlasses.Components
{
    [RequireComponent(typeof(ArgButton))]
    public class ButtonStyleEnforcer : StyleEnforcerBase<ArgButton, ButtonViewModel>
    {
        [ButtonStyle(ButtonStyleAttribute.PropertyUsage.Context)]
        [SerializeField] protected ButtonStyle _style;
        public ButtonStyle Style => _style;

        [SerializeField] private Use _use;

        public bool Stateful;
        
        [SerializeField] private StateMachineDefinition _customColorDefinition;
        
        private ButtonView _buttonView;
        
        protected override GameObject GetPrefab()
        {
            if(_use == Use.Custom) _style.Stateful = Stateful;
            return ViewCollectionManager.ButtonCollection.TryGetPrefabForStyle(_style, _use, out var prefab) ? prefab : null;
        }

        protected override void PopulateUse()
        {
            if (_buttonView == null) _buttonView = GetComponentInChildren<ButtonView>();

            if (ViewCollectionManager.ButtonStateMachineCollection.TryGetStateMachineForUse(_use, out var smd))
            {
                _buttonView.SetUse(_use, smd);
            }
            else if (_use == Use.Custom)
            {
                _buttonView.SetUse(_use, _customColorDefinition);
                _buttonView.SetStateful(_style.Stateful);
            }
            else
            {
                Debug.LogWarning($"Use SMD for {_use} not found.");
            }
        }

        public void SetStyle(ButtonStyle style)
        {
            _use = style.Use;
            _style = style;
        }
        
        public void SetStyleAndPopulate(ButtonStyle style)
        {
            SetStyle(style);
            PopulatePrefab();
        }
    }
}
