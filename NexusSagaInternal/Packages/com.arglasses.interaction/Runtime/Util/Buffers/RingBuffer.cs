using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class RingBuffer<TValue>
    {
        public TValue[] array;
        public int head;
        public int count;

        public RingBuffer(int size)
        {
            array = new TValue[size];
            head = 0;
            count = 0;
        }

        public ref TValue Append(TValue value)
        {
            int index;
            var bufferSize = array.Length;
            if (count < bufferSize)
            {
                Debug.Assert(head == 0, "Head can't have moved if buffer isn't full yet");
                index = count;
                ++count;
            }
            else
            {
                // Buffer is full. Bump head.
                index = (head + count) % bufferSize;
                ++head;
            }

            array[index] = value;
            return ref array[index];
        }

        public ref TValue this[int index]
        {
            get
            {
                if (index < 0 || index >= count)
                    throw new ArgumentOutOfRangeException(nameof(index));
                return ref array[(head + index) % array.Length];
            }
        }
    }
}
