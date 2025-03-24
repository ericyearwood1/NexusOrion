using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CTRL.Data
{
  [System.Serializable]
  public struct SettingsPayload
  {
    public string settingsJson;
  }

  public class SettingsStream : OutputStreamHandle<SettingsPayload>
  {
    protected override string defaultStreamName => "settings";
  }
}
