using UnityEngine;

namespace ARGlasses.Interaction
{
    public enum BubbleEventHandlePolicy
    {
        None, // not yet handled
        Continue, // was handled by something, but will continue to route to other handlers.
        StopBubbling, // components on the same GO will still receive it.
        StopPropagation, // it's handled, and no other component should handle it.
    }

    public class BubbleEvent
    {
        public BubbleEvent(Component target) {
            Target = target;
        }

        public Component Target { get; private set; }

        public BubbleEventHandlePolicy HandlePolicy { get; private set; } = BubbleEventHandlePolicy.None;

        public void Handle(BubbleEventHandlePolicy mode)
        {
            // it can only get more strict
            if (mode > HandlePolicy)
                HandlePolicy = mode;
        }
    }
}
