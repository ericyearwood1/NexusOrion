using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CTRL
{

  // To be deserialized automatically
  [System.Serializable]
  public struct DeviceMonitorData
  {
    public bool live;
    public float time_since_data;
  }


  public class DeviceMonitorStream : OutputStreamHandle<DeviceMonitorData>
  {
    // The stream name in your YAML (can also be set in the editor)
    protected override string defaultStreamName => "device_monitor.main";
  }

}
