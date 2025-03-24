using OSIG.Tools.StateMachines;
using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "ButtonUseSMDs", menuName = "FigmaUnity/ARDS/Button Use SMD Collection", order = 0)]
    public class ButtonUseStateMachineCollection : ScriptableObject
    {
        [SerializeField] private GenericDictionary<Use, StateMachineDefinition> _useToStateMachines;
        public static readonly string ResourcesPath = "ButtonUseSMDs";

        public bool TryGetStateMachineForUse(Use use, out StateMachineDefinition definition)
        {
            return _useToStateMachines.TryGetValue(use, out definition);
        }
    }
}
