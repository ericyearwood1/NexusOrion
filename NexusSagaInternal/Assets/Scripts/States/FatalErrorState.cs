using Data;
using Prime31.StateKit;
using UnityEngine;

namespace States
{
    public class FatalErrorState : SKState<AppData>
    {
        public override void begin()
        {
            base.begin();
            Debug.LogError($"Fatal error state {_context.FatalError}");
            _context.FullFocusCanvasUI.ShowFatalErrorState(_context.FatalError);
        }

        public override void update(float deltaTime)
        {
        }
    }
}