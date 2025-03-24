using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;

namespace CTRL.Data
{
  public class AnnotationEventStream : InputStreamHandle<CTRLAnnotationEvent<JToken>>
  {
    protected override string defaultStreamName => "annotations";

    public void SendAnnotationEvent(string name, string @event, string payload_type, JToken payload)
    {
      SendAnnotationEvent(new CTRLAnnotationEvent<JToken>
      {
        name = name,
        @event = @event,
        payload_type = payload_type,
        payload = payload,
        generation_time = Timestamp()
      });
    }

    public void SendAnnotationEvent(CTRLAnnotationEvent<JToken> sample)
    {
      if (State == StreamState.Connected)
      {
        SendSample(sample);
      }
    }
  }
}
