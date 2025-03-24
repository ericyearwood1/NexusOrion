namespace CTRL.Data
{
  [System.Serializable]
  public struct CeresEventPayload
  {
    public string finger;
    public string action;
  }
  public class CeresEventStream : OutputStreamHandle<CeresEventPayload>
  {
    protected override string defaultStreamName => "ceres_gesture";
  }
}
