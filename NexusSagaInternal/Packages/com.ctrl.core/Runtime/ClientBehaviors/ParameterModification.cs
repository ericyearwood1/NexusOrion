
using UnityEngine;

using CTRL.Data;

namespace CTRL.ClientBehaviors
{
  [RequireComponent(typeof(ParameterMonitorStream))]
  public class ParameterModification : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected ParameterMonitorStream stream;
    public ParameterMonitorStream Stream { get => stream; }

    [SerializeField]
    protected string transformerFilter = "*";
    public string TransformerFilter { get => transformerFilter; }
  }
}
