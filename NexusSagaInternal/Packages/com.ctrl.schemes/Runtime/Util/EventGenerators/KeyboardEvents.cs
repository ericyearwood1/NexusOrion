using UnityEngine;
using UnityEngine.Events;

using System;
using System.Collections.Generic;

namespace CTRL.Utils.EventGenerators
{
  [Serializable]
  public struct KeyPress
  {
    public KeyCode Key;
    public bool Down;
  }

  [Serializable]
  public class KeyPressEmitter : UnityEvent<KeyPress> { }

  public class KeyboardEvents : ITK.DependBehavior
  {
    [SerializeField]
    protected List<KeyCode> keys = new List<KeyCode>();

    [SerializeField]
    protected KeyPressEmitter onKeyPress = new KeyPressEmitter();
    public KeyPressEmitter OnKeyPress { get => onKeyPress; }

    protected void Update()
    {
      foreach (var key in keys)
      {
        if (Input.GetKeyDown(key))
        {
          OnKeyPress.Invoke(new KeyPress { Key = key, Down = true });
        }

        if (Input.GetKeyUp(key))
        {
          OnKeyPress.Invoke(new KeyPress { Key = key, Down = false });
        }
      }
    }
  }
}
