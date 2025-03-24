using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class EventBubbler
    {
        private static readonly List<Component> route = new();

        public static void Bubble<T, E>(E eventArgs, Action<T, E> callback) where E : BubbleEvent
        {
            route.Clear();

            var element = eventArgs.Target;
            while (element != null)
            {
                route.Add(element);
                var parent = element.transform.parent;
                if (parent == null) break;
                element = parent;
            }

            foreach (var bubbleTarget in route)
            {
                foreach (var handler in bubbleTarget.GetComponents<T>())
                {
                    if (handler is not MonoBehaviour { isActiveAndEnabled: true }) continue;
                    try
                    {
                        callback(handler, eventArgs);
                    }
                    catch (Exception e)
                    {
                        Debug.LogError($"Unhandled exception from {nameof(EventBubbler)} event handler on {bubbleTarget.gameObject.name}\n{e}", bubbleTarget.gameObject);
                    }

                    if (eventArgs.HandlePolicy >= BubbleEventHandlePolicy.StopPropagation) return;
                }

                if (eventArgs.HandlePolicy >= BubbleEventHandlePolicy.StopBubbling) return;
            }
        }
    }
}
