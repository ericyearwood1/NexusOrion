using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CTRL
{

  // To be deserialized automatically
  [System.Serializable]
  public struct PliData
  {
    public int[] count;
  }


  public class PliStream : OutputStreamHandle<PliData>
  {
    // The stream name in your YAML (can also be set in the editor)
    protected override string defaultStreamName => "onlinepli.agg_counts";
  }

}
