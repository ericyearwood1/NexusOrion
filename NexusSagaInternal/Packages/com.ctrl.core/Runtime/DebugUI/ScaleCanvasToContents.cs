using UnityEngine;

namespace CTRL.DebugUI
{
  public class ScaleCanvasToContents : MonoBehaviour
  {
    [SerializeField]
    protected RectTransform _driver;

    protected RectTransform _localRect;

    protected virtual void OnEnable()
    {
      _localRect = GetComponent<RectTransform>();
      DoResize();
    }

    protected virtual void Update()
    {
      DoResize();
    }

    protected void DoResize()
    {
      if (_driver == null)
      {
        return;
      }

      _localRect.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, _driver.rect.width);
      _localRect.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, _driver.rect.height);
    }
  }
}
