using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CTRL
{

  // To be deserialized automatically
  [System.Serializable]
  public struct BatteryData
  {
    public int percent;
  }


  public class BatteryStream : OutputStreamHandle<BatteryData>
  {
    // The stream name in your YAML (can also be set in the editor)
    protected override string defaultStreamName => "device.battery";
  }

}
