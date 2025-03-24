using UnityEngine.Serialization;

namespace ARGlasses.Components.Scroll
{
    using System.Collections.Generic;
    using UnityEngine;

    public class ListItemPool : MonoBehaviour
    {
        [SerializeField] private ArgListItem _argListItemPrefab;
        [SerializeField] private Transform _inactiveGoHolder;
        private Queue<ArgListItem> pool = new Queue<ArgListItem>();

        public ArgListItem Get()
        {
            if (pool.Count == 0)
                GrowPool();

            return pool.Dequeue();
        }

        public void Return(ArgListItem item)
        {
            item.gameObject.SetActive(false);
            item.gameObject.transform.SetParent(_inactiveGoHolder);
            pool.Enqueue(item);
        }

        private void GrowPool()
        {
            ArgListItem instance = Instantiate(_argListItemPrefab);
            instance.gameObject.SetActive(false);
            pool.Enqueue(instance);
        }
    }
}
