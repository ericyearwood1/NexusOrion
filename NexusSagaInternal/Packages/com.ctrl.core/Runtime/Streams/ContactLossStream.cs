using System.Collections;
using System.Collections.Generic;
using System.Runtime.Serialization;
using UnityEngine;

using Newtonsoft.Json;
using Newtonsoft.Json.Converters;
using Newtonsoft.Json.Utilities;

namespace CTRL
{
  [JsonConverter(typeof(StringEnumConverter))]
  public enum ContactLossState
  {
    [EnumMember(Value = "contact_loss")]
    ContactLoss,

    [EnumMember(Value = "nominal")]
    Nominal,
  }
  // To be deserialized automatically
  [System.Serializable]
  public struct ContactLossData
  {
    // Either 'nominal' or 'contact_loss'
    public ContactLossState state;
    public int[] channels;
  }


  public class ContactLossStream : OutputStreamHandle<ContactLossData>
  {
    // The stream name in your YAML (can also be set in the editor)
    protected override string defaultStreamName => "contact_loss";
  }

}
