using System.Collections.Generic;
using UnityEngine;

namespace Multiplayer.Runtime.Pools
{
    public class GameObjectPool
    {
        private Queue<GameObject> _pool;

        public void Initialise(GameObject prefab, int size)
        {
            _pool = new Queue<GameObject>(size);
            for (var i = 0; i < size; ++i)
            {
                var instance = GameObject.Instantiate(prefab);
                instance.SetActive(false);
                _pool.Enqueue(instance);
            }
        }

        public GameObject Get()
        {
            var obj = _pool.Dequeue();
            obj.SetActive(true);
            return obj;
        }

        public void Return(GameObject obj)
        {
            obj.SetActive(false);
            _pool.Enqueue(obj);
        }
    }
}