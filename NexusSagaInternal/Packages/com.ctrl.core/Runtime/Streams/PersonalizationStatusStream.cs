using System.Collections.Generic;

namespace CTRL.Data
{

  [System.Serializable]
  public struct PersonalizationStatusData
  {
    public PersonalizationStatus status;
    public string msg;
    public string[] protocols;
    public string path;
    public string[] artifacts;
    public bool is_new;
    public bool is_old;
    public float progress;
    public float? training_finished;
  }

  [System.Serializable]
  public enum PersonalizationStatus
  {
    Ready,
    Available,
    Pending,
    NoData,
    Error,
    Unknown
  }

  public class PersonalizationStatusStream : OutputStreamHandle<PersonalizationStatusData>
  {
    protected override string defaultStreamName => "personalization_status";
  }
}

