using System;
using System.Collections.Generic;
using ARGlasses.Interaction;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using OSIG.Tools.Units;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components.Scroll
{
    public abstract class PoolingScroller : OCLayoutComponentFixedSizeWithPadding<OCLayoutInset.AttachStruct,
        PoolingScrollerAttachData>
    {
        public OCValue ElementHeight;
        public OCValue ScrollDistance;
        public OCValue Spacing;
        public float WindowAnimationScalar = 2;
        public int SwapSpaceScalar = 1;
        public float PaginationDistance => ElementHeightMeters + SpacingMeters;
        public float ElementHeightMeters => ElementHeight.GetMeters(UnitsContext);
        public float SpacingMeters => Spacing.GetMeters(UnitsContext);

        public abstract float Max { get; }

        public abstract override void ArrangeChildren(Pass4Data<OCLayoutInset.AttachStruct> children,
            LayoutTransform transform);

        public int FullyVisibleElementCount = 3;


        [SerializeField] protected MockListData _listData;

        protected List<ListItemViewModel> _listItemViewModels;

        public List<ListItemViewModel> ListItemViewModels
        {
            get
            {
                if (_listData == null && _listItemViewModels == null)
                    _listItemViewModels = Resources.Load<MockListData>("MockListData").ListItemViewModels;
                else if (_listData != null)
                {
                    _listItemViewModels = _listData.ListItemViewModels;
                }
                return _listItemViewModels;
            }
            protected set => _listItemViewModels = value;
        }

        public OCValue Width;
        public OCValue Depth;
        public override Vector3 GetFixedSize()
        {
            return new Vector3(Width.GetMeters(UnitsContext),FullyVisibleElementCount * (ElementHeightMeters + SpacingMeters),
                Depth.GetMeters(UnitsContext));
        }


        protected Vector3 _mySizeMeters;
        public Vector3 MySizeMeters => _mySizeMeters;

        protected int Mod(int x, int m)
        {
            int r = x % m;
            return r < 0 ? r + m : r;
        }

        protected float CalculateNormalizedMiddlePosition(float childMiddlePosition, float parentSize)
        {
            float normalizedMiddlePosition = (childMiddlePosition + parentSize / 2f) / parentSize;
            normalizedMiddlePosition = (normalizedMiddlePosition * 2 - 1) / WindowAnimationScalar;
            return normalizedMiddlePosition;
        }

        protected float CalculateNormalizedSwapPosition(float childMiddlePosition, float parentSize,
            float swapSpaceSize)
        {
            // Adjust the child's middle Y position to be relative to the parent's bottom edge
            float adjustedPosition = childMiddlePosition + parentSize / 2f;

            // If the child's middle position is within the parent's boundaries, return 0
            if (adjustedPosition >= 0 && adjustedPosition <= parentSize)
            {
                return 0f;
            }
            // If the child's middle position is above the parent's top edge
            else if (adjustedPosition > parentSize)
            {
                // Normalize the adjusted position to the range [0, 1], where 0 represents the parent's top edge and
                // 1 represents the top edge of the swap space
                return (adjustedPosition - parentSize) / swapSpaceSize;
            }
            // If the child's middle position is below the parent's bottom edge
            else
            {
                // Normalize the adjusted position to the range [0, -1], where 0 represents the parent's bottom edge and
                // -1 represents the bottom edge of the swap space
                return (adjustedPosition / (-swapSpaceSize)) * -1;
            }
        }
    }
}
