// Initial version taken from https://www.internalfb.com/code/fbsource/[ce494d07d7a4bd401cbcf5e8d8b2b989b8b0b62b]/arvr/projects/nimble/integrations/NimbleUnity/Assets/ThirdParty/IngameDebugConsole/Scripts/CircularBuffer.cs
// Functionality added on top of the above: Capacity and Enumerators

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class CircularBuffer<T> : IList<T>
    {
        [SerializeField, ReadOnly] private T[] _arr;
        [SerializeField, ReadOnly] private int _offset;
        [SerializeField, ReadOnly] private int _capacity;
        [SerializeField, ReadOnly] private int _count;

        public int Count
        {
            get => _count;
            private set => _count = value;
        }

        public int Capacity => _capacity;

        public bool IsReadOnly => false;

        public T this[int idx]
        {
            get => _arr[(_offset + idx) % _arr.Length];
            set => _arr[(_offset + idx) % _arr.Length] = value;
        }

        public CircularBuffer(int capacity)
        {
            _capacity = capacity;
            _arr = new T[Capacity];
        }

        public T Latest() => this[^1];

        public void Clear()
        {
            _arr = new T[Capacity];
            _offset = 0;
        }

        public void Add(T value)
        {
            if (Count < _arr.Length)
            {
                _arr[_offset + Count] = value;
                Count++;
            }
            else
            {
                _arr[_offset] = value;
                if (++_offset >= _arr.Length) _offset = 0;
            }
        }

        public void Insert(int idx, T val)
        {
            if (Count == Capacity) throw new IndexOutOfRangeException("Buffer already at capacity");
            if (idx > Count) throw new IndexOutOfRangeException();
            for (int i = idx; i < Count - 1; ++i) _arr[(_offset + i + 1) % _arr.Length] = _arr[(_offset + i) % _arr.Length];
            _arr[idx] = val;
            Count++;
        }

        public bool Remove(T query)
        {
            int idx = IndexOf(query);
            if (idx < 0) return false;
            RemoveAt(idx);
            return true;
        }

        public void RemoveAt(int idx)
        {
            for (int j = idx; j < Count - 1; ++j) _arr[(_offset + j) % _arr.Length] = _arr[(_offset + j + 1) % _arr.Length];
            if (idx == 0) _offset = (_offset + 1) % _arr.Length;
            Count--;
        }

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        public IEnumerator<T> GetEnumerator()
        {
            for (int i = 0; i < Count; ++i) yield return this[i];
        }

        public bool Contains(T query) => IndexOf(query) >= 0;

        public int IndexOf(T query)
        {
            for (int i = 0; i < Count; ++i)
                if (this[i].Equals(query))
                    return i;
            return -1;
        }

        public void CopyTo(T[] array, int arrayIndex)
        {
            if (array == null) throw new ArgumentNullException("The array cannot be null.");
            if (arrayIndex < 0) throw new ArgumentOutOfRangeException("The starting array index cannot be negative.");
            if (Count > array.Length - arrayIndex) throw new ArgumentException("The destination array has fewer elements than the collection.");
            for (int i = 0; i < Count; ++i) array[i + arrayIndex] = this[i];
        }
    }

    public class CircularBufferTest : MonoBehaviour
    {
        [SerializeField] private int _nextValue = 0;
        [SerializeField] private CircularBuffer<int> _buffer = new CircularBuffer<int>(10);
        [SerializeField, ReadOnly] private int _lastValue;

        private void OnEnable()
        {
            _buffer = new CircularBuffer<int>(10);
        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.Space))
            {
                _buffer.Add(_nextValue);
                _nextValue++;
            }

            if (_buffer.Count > 0) _lastValue = _buffer[^1];
        }
    }
}
