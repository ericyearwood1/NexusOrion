using System;
using TMPro;
using UnityEngine;

public class TextPillResizer : MonoBehaviour
{
    public TextMeshPro pillText;
    public SpriteRenderer pillSpriteRenderer;
    public Transform lineSquare;
    public Transform headPositioner;
    public float paddingX = 0.5f;
    public float maxWidth = 10.0f;
    public float averageRatio = 0.0323f; 
    public float minPillScale = 0.1f;
    public float pillUnitSize = 1.85f;

    private void Start()
    {
        AdjustTextWidthAndPillSize();
    }

    [ContextMenu("Adjust Text and Pill Size")]
    private void AdjustTextWidthAndPillSize()
    {
        if (pillText == null || pillSpriteRenderer == null) return;
        
        pillText.ForceMeshUpdate();

        float preferredWidth = pillText.preferredWidth + paddingX;
        preferredWidth = Mathf.Min(preferredWidth, maxWidth);

        RectTransform rectTransform = pillText.GetComponent<RectTransform>();
        rectTransform.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, preferredWidth);

        float newPillScale = averageRatio * preferredWidth;
        newPillScale = Mathf.Max(newPillScale, minPillScale);

        pillSpriteRenderer.transform.localScale = new Vector3(newPillScale, newPillScale, 1f);
        
        // Resize and reposition the line renderer
        if (headPositioner != null && lineSquare != null)
        {
            float positionY = headPositioner.transform.localPosition.y;
            lineSquare.localPosition = new Vector3(0, positionY * 0.5f - pillUnitSize * newPillScale * 0.25f, 0);
            lineSquare.localScale = new Vector3(0.01f, positionY - pillUnitSize * newPillScale * 0.5f, 0);
        }
    }
}