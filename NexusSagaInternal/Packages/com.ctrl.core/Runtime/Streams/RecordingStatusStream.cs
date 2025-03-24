using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CTRL.Data
{
  [System.Serializable]
  public enum RecordingStatus
  {
    Uploaded,
    Uploading
  }

  [System.Serializable]
  public struct RecordingStatusEvent
  {
    public RecordingStatus status;
    public string ds_key;
    public float? progress;
  }

  public class RecordingStatusStream : OutputStreamHandle<RecordingStatusEvent>
  {
    protected override string defaultStreamName => "recording_status";
  }
}
