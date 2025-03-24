using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;

namespace CTRL.Data
{
  [System.Serializable]
  public class Metadata
  {
    public JToken session = new JObject();
    public JToken devices = new JObject();
  }

  [System.Serializable]
  public struct RecorderControlMessage
  {
    // [JsonProperty(Required = Required.Always)]
    public bool enabled;

    // [JsonProperty(Required = Required.Always)]
    public bool verbose;
    public Metadata metadata;
  }

  public class RecordingControlStream : InputStreamHandle<RecorderControlMessage>
  {
    protected override string defaultStreamName => "recording_control";

    public void StartRecording()
    {
      SendRecorderControlMessage(true, true, new Metadata());
    }

    public void StopRecording()
    {
      SendRecorderControlMessage(false, true, new Metadata());
    }

    public void SendMetadata(Metadata metadata)
    {
      SendRecorderControlMessage(true, true, metadata);
    }

    public void SendRecorderControlMessage(bool enabled, bool verbose, Metadata metadata)
    {
      if (State == StreamState.Connected)
      {
        SendSample(new RecorderControlMessage
        {
          enabled = enabled,
          verbose = verbose,
          metadata = metadata,
        });
      }
    }
  }
}
