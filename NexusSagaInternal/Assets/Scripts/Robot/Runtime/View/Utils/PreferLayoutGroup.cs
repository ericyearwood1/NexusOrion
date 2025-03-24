using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

[RequireComponent(typeof(HorizontalOrVerticalLayoutGroup))]
[RequireComponent(typeof(RectTransform))]
public class PreferLayoutGroup : UIBehaviour, ILayoutElement
{
    [SerializeField] private float _minHeight = 290f;
    [SerializeField] private float _minWidth = 500f;
    public float minWidth
    {
        get
        {
            return _minWidth;
        }
    }

    public float preferredWidth
    {
        get
        {
            return GetLayoutGroup().preferredWidth;
        }
    }

    public float flexibleWidth
    {
        get
        {
            return GetLayoutGroup().flexibleWidth;
        }
    }

    public float minHeight
    {
        get
        {
            return _minHeight;
        }
    }

    public float preferredHeight
    {
        get
        {
            return GetLayoutGroup().preferredHeight;
        }
    }

    public float flexibleHeight
    {
        get
        {
            return GetLayoutGroup().preferredHeight;
        }
    }

    public int layoutPriority
    {
        get
        {
            return 100;
        }
    }

    ILayoutElement layoutGroup;

    public void CalculateLayoutInputHorizontal()
    {
    }

    public void CalculateLayoutInputVertical()
    {
    }

    protected override void OnEnable()
    {
        base.OnEnable();
        LayoutRebuilder.MarkLayoutForRebuild(GetComponent<RectTransform>());
    }

    protected override void OnDisable()
    {
        LayoutRebuilder.MarkLayoutForRebuild(GetComponent<RectTransform>());
        base.OnDisable();
    }

    ILayoutElement GetLayoutGroup()
    {
        if (layoutGroup == null)
        {
            layoutGroup = GetComponent<HorizontalOrVerticalLayoutGroup>();
        }
        return layoutGroup;
    }
}