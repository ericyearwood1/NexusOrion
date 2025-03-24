using System.Collections;
using System.Collections.Generic;
using ARGlasses.Components;
using UnityEngine;
using OSIG.Tools.Units;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;

public class
    OCLayoutSlider : OCLayoutComponentFlexibleSizeWithPadding<OCSliderAttachData.AttachStruct, OCSliderAttachData>
{
    [Range(0, 1)] public float Value;

    public HandleShapeType HandleShape = HandleShapeType.Square;
    public Orientation Orientation = Orientation.Horizontal;

    public OCValue HandleWidth;

    public override void ArrangeChildren(Pass4Data<OCSliderAttachData.AttachStruct> children, LayoutTransform transform)
    {
        Vector3 size = transform.MaxCorner - transform.MinCorner;


        float handleWidth;
        if (HandleShape == HandleShapeType.Square)
        {
            handleWidth = Orientation == Orientation.Horizontal
                ? transform.MaxCorner.y - transform.MinCorner.y
                : transform.MaxCorner.x - transform.MinCorner.x;
        }
        else
        {
            handleWidth = HandleWidth.GetMeters(UnitsContext);
        }

        float unusedSpace =
            (Orientation == Orientation.Horizontal
                ? transform.MaxCorner.x - transform.MinCorner.x
                : transform.MaxCorner.y - transform.MinCorner.y) - handleWidth;

        float leftFill = Value * unusedSpace;
        float rightFill = (1 - Value) * unusedSpace;

        Vector3 localMin = transform.MinCorner - transform.LocalPosition;
        Vector3 localMax = transform.MaxCorner - transform.LocalPosition;

        var normalizedDirection = Orientation == Orientation.Horizontal ? Vector3.right : Vector3.up;

        for (int i = 0; i < children.Length; i++)
        {
            switch (children.Data[i].Type)
            {
                case OCSliderAttachData.FillType.Handle:
                    children.Transforms[i] = LayoutTransform.FromLocalCorners(localMin + normalizedDirection * leftFill,
                        localMax - normalizedDirection * rightFill);
                    break;
                case OCSliderAttachData.FillType.InclusiveFill:
                    children.Transforms[i] =
                        LayoutTransform.FromLocalCorners(localMin, localMax - normalizedDirection * rightFill);
                    break;
                case OCSliderAttachData.FillType.ExclusiveFill:
                    children.Transforms[i] = LayoutTransform.FromLocalCorners(localMin,
                        localMax - normalizedDirection * (rightFill + handleWidth));
                    break;
            }
        }
    }

    public enum HandleShapeType
    {
        Square,
        Width
    }
}
