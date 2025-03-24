using System.Collections.Generic;
using UnityEngine;

namespace Multiplayer.Runtime.Pools
{
    public class UserPool<T> where T : MonoBehaviour
    {
        private Queue<T> _pool;

        public void Initialise(GameObject prefab, int size, Transform container = null)
        {
            _pool = new Queue<T>(size);
            for (var i = 0; i < size; ++i)
            {
                var instance = GameObject.Instantiate(prefab);
                if (container != null)
                {
                    instance.transform.parent = container;
                    instance.transform.localPosition = Vector3.zero;
                    instance.transform.localScale = Vector3.one;
                    instance.transform.localRotation = Quaternion.identity;
                }
                var component = instance.GetComponent<T>();
                component.gameObject.SetActive(false);
                _pool.Enqueue(component);
            }
        }

        public T Get()
        {
            var obj = _pool.Dequeue();
            obj.gameObject.SetActive(true);
            return obj;
        }

        public void Return(T obj)
        {
            obj.gameObject.SetActive(false);
            _pool.Enqueue(obj);
        }
    }
}