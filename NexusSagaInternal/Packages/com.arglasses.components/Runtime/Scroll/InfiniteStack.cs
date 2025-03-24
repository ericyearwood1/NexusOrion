using System;
using System.Collections.Generic;
using System.Linq;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using OSIG.Tools.Units;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components.Scroll
{
    public class InfiniteStack : OCLayoutComponentFlexibleSizeWithPadding
    {
        public float Scroll;
        public int VisibleCount = 3;
        public OCValue ChildHeight;
        public OCValue ChildWidth;

        private List<ChildData> _sortedChildrenData;

        public override void ArrangeChildren(Pass4Data<OCLayoutInset.AttachStruct> children,
            LayoutTransform layoutTransform)
        {
            //calculate visible height extents. visible count * size of each.
            //We can assume all are the same size for now
            //we add 2 to account for the top edge/bottom edge still being visible
            var visibleHeight = (VisibleCount +2)* ChildHeight.GetMeters(UnitsContext);
            var normalizedScroll = Scroll - Mathf.Floor(Scroll);
            layoutTransform = LayoutTransform.PositionWithSize(layoutTransform.LocalPosition,
                new Vector3(ChildWidth.GetMeters(UnitsContext), visibleHeight));


            for (int i = 0; i < children.Length; i++)
            {

                var yPos = Mathf.Lerp(layoutTransform.LocalPosition.y - visibleHeight / 2f,
                    layoutTransform.LocalPosition.y + visibleHeight / 2f, normalizedScroll);
                var newPos = new Vector3(layoutTransform.LocalPosition.x, yPos, layoutTransform.LocalPosition.z);
                var size = children.FinalSizes[i];
                children.Transforms[i] = LayoutTransform.PositionWithSize(newPos, size);
            }

            // // Find center of viewport
            // float viewportCenter = layoutTransform.LocalPosition.y;
            //
            // // Calculate distance from each child to the center of the viewport and sort by distance
            // _sortedChildrenData = children.Transforms.AsEnumerable(children.Length)
            //     .Select(child => new ChildData
            //     {
            //         Child = LayoutTransform.PositionWithSize(child.LocalPosition, child.MaxCorner - child.MinCorner),
            //         Distance = Mathf.Abs(viewportCenter - child.LocalPosition.y)
            //     })
            //     .OrderBy(item => item.Distance)
            //     .ToList();
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.green;

            var size = new Vector3(CurrentSize.x, ChildHeight.GetMeters(UnitsContext) * VisibleCount,CurrentSize.z);

            Gizmos.DrawWireCube(GetRectTransform().position,size);
            // for (var i = 0; i < _sortedChildrenData.Count; i++)
            // {
            //     Vector3 childSize = _sortedChildrenData[i].Child.MaxCorner - _sortedChildrenData[i].Child.MinCorner;
            //
            //     var visibleColor = Color.Lerp(Color.green, Color.yellow, (float)i / VisibleCount);
            //     Gizmos.color = i < VisibleCount ? visibleColor : Color.red;
            //
            //     Gizmos.DrawWireCube(GetRectTransform().position + _sortedChildrenData[i].Child.LocalPosition, childSize);
            // }
        }

        private struct ChildData
        {
            public LayoutTransform Child { get; set; }
            public float Distance { get; set; }
        }
    }
}
