using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class HierarchySubscriber<T> where T : MonoBehaviour
    {
        private static void CheckStaticReload()
        {
            if (_lastFrame > Time.frameCount)
            {
                Debug.Log("Clearing HierarchySubscriber");
                _all.Clear();
            }
            _lastFrame = Time.frameCount;
        }

        private static int _lastFrame;
        private static List<HierarchySubscriber<T>> _all = new();
        public static IReadOnlyList<HierarchySubscriber<T>> All
        {
            get
            {
                CheckStaticReload();
                return _all;
            }
        }

        [SerializeField] public RelationFlags _relationFlags = RelationFlags.Ancestor;
        [SerializeField, ReadOnly] private List<T> _publishers = new();
        public IReadOnlyList<T> Publishers => _publishers;

        public event Action<T, bool> WhenActiveRelativesChanged = delegate { };

        private Component _component;
        public HierarchySubscriber(Component component)
        {
            CheckStaticReload();
            _component = component;

            _all.Add(this);
            foreach (var publisher in HierarchyPublisher<T>.All) publisher.Notify(this);
        }

        public void Destroy()
        {
            _component = null;
            _all.Remove(this);
        }

        public HierarchySubscriber(Component component, RelationFlags relationFlags) : this(component)
        {
            _relationFlags = relationFlags;
        }

        public void Notify(T publisher, bool isActive)
        {
            if (isActive)
            {
                if (_component.IsRelated(publisher, _relationFlags) && !_publishers.Contains(publisher))
                {
                    _publishers.Add(publisher);
                    WhenActiveRelativesChanged(publisher, true);
                }
            }
            else
            {
                if (_publishers.Contains(publisher))
                {
                    _publishers.Remove(publisher);
                    WhenActiveRelativesChanged(publisher, false);
                }
            }
        }
    }
}
