namespace CTRL.ClientBehaviors
{
  public class ExecutionOrder
  {
    public const int LoadConfigFile = 1; // Must execute before ScanForHost
    public const int ScanForHost = 2; // Must execute after LoadConfigFile
  }
}
