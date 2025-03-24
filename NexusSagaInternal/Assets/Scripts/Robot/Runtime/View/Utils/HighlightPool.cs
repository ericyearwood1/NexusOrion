using System.Collections.Generic;
using Robot.Runtime.View;
using UnityEngine;

namespace Robot.Runtime.Utils
{
    public class HighlightPool<T> where T : MonoBehaviour, IResettable
    {
        private Queue<T> _pool;
        private Transform _poolContainer;

        public HighlightPool(Transform container, int poolSize, GameObject prefab)
        {
            _poolContainer = container;
            _pool = new Queue<T>(poolSize);
            for (var i = 0; i < poolSize; ++i)
            {
                var go = GameObject.Instantiate(prefab, _poolContainer, false);
                go.transform.localPosition = Vector3.zero;
                go.transform.localScale = Vector3.one;
                var icon = go.GetComponent<T>();
                _pool.Enqueue(icon);
                go.SetActive(false);
            }
        }

        public T Get()
        {
            var iconView = _pool.Dequeue();
            iconView.transform.localScale = Vector3.one;
            iconView.gameObject.SetActive(true);
            return iconView;
        }
        
        public void Return(T iconView)
        {
            iconView.transform.SetParent(_poolContainer);
            iconView.Reset();
            iconView.gameObject.SetActive(false);
            _pool.Enqueue(iconView);
        }
    }
}