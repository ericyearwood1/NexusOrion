using System;
using UnityEngine;
using UnityEngine.Events;

namespace CTRL.Utils.EventGenerators
{
  [Serializable]
  public struct MouseButtonAction
  {
    public int Button;
    public bool Down;
  }

  [Serializable]
  public class Vector2Emitter : UnityEvent<Vector2> { }

  [Serializable]
  public class MouseButtonEmitter : UnityEvent<MouseButtonAction> { }

  public class MouseEvents : ITK.DependBehavior
  {
    [SerializeField]
    protected Vector2Emitter onMouseCursor = new Vector2Emitter();
    public Vector2Emitter OnMouseCursor { get => onMouseCursor; }

    [SerializeField]
    protected MouseButtonEmitter onMouseButton = new MouseButtonEmitter();
    public MouseButtonEmitter OnMouseButton { get => onMouseButton; }

    [SerializeField]
    protected Vector2Emitter onMouseScroll = new Vector2Emitter();
    public Vector2Emitter OnMouseScroll { get => onMouseScroll; }

    protected void Update()
    {
      Vector3 pt = Input.mousePosition;
      pt.Scale(new Vector3(1f / Screen.width, 1f / Screen.height, 1));

      Vector2 screenPt = new Vector2(pt.x * 2 - 1, pt.y * 2 - 1);
      OnMouseCursor.Invoke(screenPt);

      for (int i = 0; i < 2; ++i)
      {
        if (Input.GetMouseButtonDown(i))
        {
          OnMouseButton.Invoke(new MouseButtonAction { Button = i, Down = true });
        }
        if (Input.GetMouseButtonUp(i))
        {
          OnMouseButton.Invoke(new MouseButtonAction { Button = i, Down = false });
        }
      }

      if (Input.mouseScrollDelta.magnitude > 0)
      {
        OnMouseScroll.Invoke(Input.mouseScrollDelta);
      }
    }
  }
}
