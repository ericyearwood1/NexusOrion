﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Components
{
    /// <summary>
    ///     Generic Serializable Dictionary for Unity 2020.1 and above.
    ///     Simply declare your key/value types and you're good to go - zero boilerplate.
    /// </summary>
    [Serializable]
    public class GenericDictionary<TKey, TValue> : IDictionary<TKey, TValue>, ISerializationCallbackReceiver
    {
        // Internal
        [SerializeField] private List<KeyValuePair> list = new();

#pragma warning disable 0414
        [SerializeField] [HideInInspector] private bool keyCollision;
#pragma warning restore 0414
        // todo SerializeField has no effect on Dictionary?
        [SerializeField] [HideInInspector] private Dictionary<TKey, TValue> dict = new();
        [SerializeField] [HideInInspector] private Dictionary<TKey, int> indexByKey = new();

        // IDictionary
        public TValue this[TKey key]
        {
            get => dict[key];
            set
            {
                dict[key] = value;
                if (indexByKey.ContainsKey(key))
                {
                    var index = indexByKey[key];
                    list[index] = new KeyValuePair(key, value);
                }
                else
                {
                    list.Add(new KeyValuePair(key, value));
                    indexByKey.Add(key, list.Count - 1);
                }
            }
        }

        public ICollection<TKey> Keys => dict.Keys;
        public ICollection<TValue> Values => dict.Values;

        public void Add(TKey key, TValue value)
        {
            dict.Add(key, value);
            list.Add(new KeyValuePair(key, value));
            indexByKey.Add(key, list.Count - 1);
        }

        public bool ContainsKey(TKey key)
        {
            return dict.ContainsKey(key);
        }

        public bool Remove(TKey key)
        {
            if (!dict.Remove(key)) return false;
            var index = indexByKey[key];
            list.RemoveAt(index);
            UpdateIndexLookup(index);
            indexByKey.Remove(key);
            return true;
        }

        public bool TryGetValue(TKey key, out TValue value)
        {
            return dict.TryGetValue(key, out value);
        }

        // ICollection
        public int Count => dict.Count;
        public bool IsReadOnly { get; set; }

        public void Add(KeyValuePair<TKey, TValue> pair)
        {
            Add(pair.Key, pair.Value);
        }

        public void Clear()
        {
            dict.Clear();
            list.Clear();
            indexByKey.Clear();
        }

        public bool Contains(KeyValuePair<TKey, TValue> pair)
        {
            if (!dict.TryGetValue(pair.Key, out var value)) return false;
            return EqualityComparer<TValue>.Default.Equals(value, pair.Value);
        }

        public void CopyTo(KeyValuePair<TKey, TValue>[] array, int arrayIndex)
        {
            if (array == null) throw new ArgumentException("The array cannot be null.");
            if (arrayIndex < 0) throw new ArgumentOutOfRangeException("The starting array index cannot be negative.");
            if (array.Length - arrayIndex < dict.Count)
                throw new ArgumentException("The destination array has fewer elements than the collection.");

            foreach (var pair in dict)
            {
                array[arrayIndex] = pair;
                arrayIndex++;
            }
        }

        public bool Remove(KeyValuePair<TKey, TValue> pair)
        {
            if (!dict.TryGetValue(pair.Key, out var value)) return false;
            if (EqualityComparer<TValue>.Default.Equals(value, pair.Value)) return Remove(pair.Key);
            return false;
        }

        // IEnumerable
        public IEnumerator<KeyValuePair<TKey, TValue>> GetEnumerator()
        {
            return dict.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return dict.GetEnumerator();
        }

        // Lists are serialized natively by Unity, no custom implementation needed.
        public void OnBeforeSerialize()
        {
        }

        // Populate dictionary with pairs from list and flag key-collisions.
        public void OnAfterDeserialize()
        {
            dict.Clear();
            indexByKey.Clear();
            keyCollision = false;
            for (var i = 0; i < list.Count; i++)
            {
                var key = list[i].Key;
                if (key != null && !ContainsKey(key))
                {
                    dict.Add(key, list[i].Value);
                    indexByKey.Add(key, i);
                }
                else
                {
                    keyCollision = true;
                }
            }
        }

        private void UpdateIndexLookup(int removedIndex)
        {
            for (var i = removedIndex; i < list.Count; i++)
            {
                var key = list[i].Key;
                indexByKey[key]--;
            }
        }

        [Serializable]
        private struct KeyValuePair
        {
            public TKey Key;
            public TValue Value;

            public KeyValuePair(TKey Key, TValue Value)
            {
                this.Key = Key;
                this.Value = Value;
            }
        }
    }
}
