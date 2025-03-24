namespace CTRL.Utils
{
  /// <summary>
  /// A first-in, first-out, thread-safe circular queue with a fixed capacity.
  /// Has the ability to be efficently copied to an array, due to storing all items contiguously.
  /// Useful both as a fast message queue, and also for storing the last-N items of a stream.
  /// </summary>
  public class SliceableFIFO<T>
  {
    readonly T[] store;

    uint startOffset = 0;
    uint items = 0;
    readonly uint capacity;

    public uint Count => this.items;
    public uint Capacity => this.capacity;

    /// Construct a FIFO with this maximum capacity. Elements added when the FIFO is full
    /// will overwrite earlier ones with a FIFO policy.
    public SliceableFIFO(uint capacity)
    {
      this.store = new T[capacity * 2];
      this.capacity = capacity;
    }

    /// Enqueue an item to the back of the queue. Will overwrite old items with `default(T)`
    /// if queue is full, like a circular buffer.
    public void Enqueue(T val)
    {
      lock (this)
      {
        if (items == capacity)
        {
          store[startOffset] = default;
          startOffset++;
          items--;
        }
        if (startOffset + items == store.Length - 1)
        {
          System.Array.Copy(store, startOffset, store, 0, items);
          startOffset = 0;
        }
        store[startOffset + items] = val;
        items++;

      }
    }

    /// Dequeue an item from the front of the queue. Throws `InvalidArgumentException` if empty.
    public T Dequeue()
    {
      lock (this)
      {
        if (items == 0)
        {
          throw new System.InvalidOperationException("CtrlLabs: SliceableFIFO: Need items > 0 to Dequeue");
        }
        else
        {
          var val = store[startOffset];
          store[startOffset] = default;
          startOffset++;
          items--;
          return val;
        }

      }
    }

    /// Copy the queue to an array.
    public void CopyTo(T[] array, uint offset = 0)
    {
      lock (this)
      {
        System.Array.Copy(store, startOffset, array, offset, items);
      }
    }

    /// Index the queue from the **front**, i.e `fifo[0] == fifo.Dequeue()`.
    public T this[uint i]
    {
      get
      {
        if (i >= items)
          throw new System.ArgumentOutOfRangeException();
        return store[startOffset + i];
      }
    }
  }
}
