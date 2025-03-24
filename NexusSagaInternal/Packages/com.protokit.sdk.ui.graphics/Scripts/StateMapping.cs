using System;

namespace ProtoKit.UI {

    /// <summary>
    /// A state mapping allows a script to be configured to use specific
    /// names for different states while interfacing with a StateMachine.
    /// </summary>
    [Serializable]
    public class StateMapping {
        public string Normal;
        public string Hovered;
        public string Pressed;
        public string Selected;
    }
}
