using Robot.Runtime.View;
using UnityEngine;

public class DetectingObjectHighlightView : LabelHighlightView
{
    [SerializeField] private SpriteRenderer _circle;

    public void Show(Transform cameraTransform, int orderIndex)
    {
        _maxAlpha = 1f/orderIndex;
        base.Show(cameraTransform, $"{orderIndex}");
    }
    
    protected override void UpdateDisplay(float progress)
    {
        base.UpdateDisplay(progress);
        _circle.color = new Color(1, 1, 1, progress * _maxAlpha);
    }
}