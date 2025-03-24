using System.Collections.Generic;

namespace CTRL.Data
{
  /// An event annotation sent as input into CTRL-R
  /// Documentation: https://fb.quip.com/9SNRAkIaXv8e
  [System.Serializable]
  public struct CTRLAnnotationEvent<T>
  {
    /// Human readable event name, e.g. 'pinch_index_press', 'thumb_swipe_north'
    public string name;
    /// One of: `start`, `end`, or `instant`
    public string @event;
    /// If payload is present, payload_type identifies its type.
    public string payload_type;

    /// Payload
    public T payload;

    /// Time of event, as determined by front-end
    public double generation_time;
  }
}
