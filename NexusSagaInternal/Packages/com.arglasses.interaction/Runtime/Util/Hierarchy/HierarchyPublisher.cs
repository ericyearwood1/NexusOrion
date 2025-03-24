using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class HierarchyPublisher<T> where T : MonoBehaviour
    {
        private static void CheckStaticReload()
        {
            if (_lastFrame > Time.frameCount)
            {
                Debug.Log("Clearing HierarchyPublisher");
                _all.Clear();
            }
            _lastFrame = Time.frameCount;
        }

        private static List<HierarchyPublisher<T>> _all = new();
        public static IReadOnlyList<HierarchyPublisher<T>> All
        {
            get
            {
                CheckStaticReload();
                return _all;
            }
        }

        private static int _lastFrame;

        private T _publisher;

        public HierarchyPublisher(T publisher)
        {
            CheckStaticReload();
            _publisher = publisher;
            _all.Add(this);
            Notify();
        }

        public void Destroy()
        {
            _publisher = null;
            _all.Remove(this);
        }

        public void Notify()
        {
            foreach (var subscriber in HierarchySubscriber<T>.All) Notify(subscriber);
        }

        public void Notify(HierarchySubscriber<T> subscriber)
        {
            subscriber.Notify(_publisher, _publisher.enabled);
        }

    }
}
