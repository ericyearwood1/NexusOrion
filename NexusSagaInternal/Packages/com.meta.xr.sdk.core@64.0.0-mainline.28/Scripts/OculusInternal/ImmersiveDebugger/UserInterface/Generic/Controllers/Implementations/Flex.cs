/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if OVR_INTERNAL_CODE

using System;
using UnityEngine;

namespace Meta.XR.ImmersiveDebugger.UserInterface.Generic
{
    public class Flex : Controller
    {
        private Vector2 _sizeDelta;

        public Vector2 SizeDelta => _sizeDelta;
        public Vector2 SizeDeltaWithMargin => _sizeDelta + LayoutStyle.TopLeftMargin + LayoutStyle.BottomRightMargin;

        internal ScrollViewport ScrollViewport { get; set; }
        private Vector2 _previousAnchoredPosition;

        public override void RefreshLayoutInternal()
        {
            base.RefreshLayoutInternal();

            if (_children == null) return;

            var direction = _layoutStyle.flexDirection switch
            {
                LayoutStyle.Direction.Left => Vector2.left,
                LayoutStyle.Direction.Right => Vector2.right,
                LayoutStyle.Direction.Down => Vector2.down,
                LayoutStyle.Direction.Up => Vector2.up,
                _ => throw new ArgumentOutOfRangeException()
            };

            var offset = Vector2.zero;
            foreach(var child in _children)
            {
                UpdateAnchoredPosition(child, ref offset, direction);
            }

            _sizeDelta = new Vector2(Mathf.Abs(offset.x), Mathf.Abs(offset.y));

            RefreshVisibilities(true);
        }

        private void UpdateAnchoredPosition(Controller controller, ref Vector2 offset, Vector2 direction)
        {
            var anchoredPosition = controller.RectTransform.anchoredPosition;
            var size = controller.RectTransform.sizeDelta;
            controller.RectTransform.anchoredPosition = anchoredPosition + offset;

            offset += direction * size;
            offset += direction * _layoutStyle.spacing;
        }

        private void RefreshVisibilities(bool force = false)
        {
            if (ScrollViewport == null) return;
            if (_children == null) return;

            var newPosition = RectTransform.anchoredPosition;
            if (!force && newPosition == _previousAnchoredPosition) return;
            _previousAnchoredPosition = newPosition;

            var scroll = RectTransform.anchoredPosition;
            var viewportRect = new Rect(ScrollViewport.RectTransform.anchoredPosition, ScrollViewport.RectTransform.rect.size);

            foreach (var child in _children)
            {
                if (IsInViewport(child, viewportRect, scroll))
                {
                    child.Show();
                }
                else
                {
                    child.Hide();
                }
            }
        }

        private bool IsInViewport(Controller controller, Rect viewportRect, Vector2 scroll)
        {
            var position = - controller.RectTransform.anchoredPosition - scroll;
            return viewportRect.Contains(position) || viewportRect.Contains(position + controller.RectTransform.sizeDelta);
        }

        protected override void AdjustHeight()
        {
            if (!LayoutStyle.adaptHeight) return;

            if (RectTransform == null) return;

            RectTransform.sizeDelta = new Vector2(RectTransform.sizeDelta.x, Mathf.Abs(_sizeDelta.y));
        }

        public void Update()
        {
            RefreshVisibilities();
        }
    }
}

#endif
