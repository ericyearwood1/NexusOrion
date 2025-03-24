using System.Collections.Generic;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using OSIG.Tools.Units;
using UnityEngine;

namespace ARGlasses.Components.Scroll
{
    public class InfinitePoolingScroller : PoolingScroller
    {

        /// <summary>
        /// for bounded version, max will be the scroll at which the last viewmodel is at the top of the rect
        /// </summary>
        public override float Max =>
            1;


        public override void ArrangeChildren(Pass4Data<OCLayoutInset.AttachStruct> children, LayoutTransform transform)
        {
            var elementHeight = ElementHeightMeters;
            var spacing = SpacingMeters;
            _mySizeMeters = transform.MaxCorner - transform.MinCorner;
            Vector3 childSize = new Vector3(_mySizeMeters.x, elementHeight, _mySizeMeters.z);

            int requiredChildren =
                Mathf.CeilToInt((_mySizeMeters.y + SwapSpaceScalar * (elementHeight + spacing)) /
                                (elementHeight + spacing)) +
                1;


            var scrollDistance = ScrollDistance.GetMeters(UnitsContext) - (elementHeight + spacing) * SwapSpaceScalar;


            if (children.Length < requiredChildren)
            {
                Debug.LogError("Not enough children to scroll with!");
                return;
            }

            int scrollElementOffset = Mathf.FloorToInt((scrollDistance - spacing) / (elementHeight + spacing));
            float offsetFraction = (scrollDistance - spacing) / (elementHeight + spacing) - scrollElementOffset;

            // Adjusting initial childCenter to consider swap space
            var childCenter = new Vector3(0, _mySizeMeters.y * 0.5f + SwapSpaceScalar * (elementHeight + spacing), 0) +
                              new Vector3(0, (offsetFraction * (elementHeight + spacing)), 0) +
                              new Vector3(0, childSize.y * -0.5f, 0);

            var attachChildren = AttachChildren;

            for (int i = 0; i < children.Length; i++)
            {
                int index = Mod((scrollElementOffset + i), children.Length);

                children.Transforms[index] = LayoutTransform.PositionWithSize(childCenter, childSize);

                float normalizedMiddlePosition = CalculateNormalizedMiddlePosition(childCenter.y, _mySizeMeters.y);
                float normalizedSwapSpacePosition = CalculateNormalizedSwapPosition(childCenter.y, _mySizeMeters.y,
                    SwapSpaceScalar * (elementHeight + spacing) / 2f);

                childCenter -= new Vector3(0, elementHeight + spacing, 0);

                int dataIndex = Mod((scrollElementOffset + i), ListItemViewModels.Count);


                attachChildren[index].SetViewModel(ListItemViewModels[dataIndex],
                    Mathf.Clamp(normalizedSwapSpacePosition * WindowAnimationScalar, -1, 1));
            }
        }
    }
}
