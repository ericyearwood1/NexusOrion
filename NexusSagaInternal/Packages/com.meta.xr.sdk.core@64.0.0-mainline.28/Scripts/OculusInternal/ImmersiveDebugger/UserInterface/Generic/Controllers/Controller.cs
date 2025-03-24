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
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Meta.XR.ImmersiveDebugger.UserInterface.Generic
{
    public class Controller : MonoBehaviour
    {
        private bool _visibility = true;

        [SerializeField]
        protected LayoutStyle _layoutStyle;
        protected List<Controller> _children;

        public Controller Owner { get; set; }
        public Transform Transform { get; protected set; }
        public RectTransform RectTransform { get; protected set; }
        protected GameObject GameObject { get; set; }
        public List<Controller> Children => _children;

        private RectMask2D _mask = null;

        public LayoutStyle LayoutStyle
        {
            get => _layoutStyle;
            set
            {
                if (_layoutStyle == value) return;

                _layoutStyle = value;
                RefreshLayout();
            }
        }

        public event Action<Controller> OnVisibilityChangedEvent;

        protected virtual void Setup(Controller owner)
        {
            Owner = owner;
            GameObject = gameObject;
            GameObject.layer = 20; // TODO : Setting
            RectTransform = GameObject.AddComponent<RectTransform>() ?? GameObject.GetComponent<RectTransform>();
            Transform = RectTransform ? RectTransform : GameObject.transform;
            if (Owner != this && Owner != null)
            {
                Transform.SetParent(Owner.Transform, false);
            }

            _layoutStyle = Style.Default<LayoutStyle>();
        }

        public T Append<T>(string childName)
            where T : Controller, new()
        {
            var childObject = new GameObject(childName);
            var childController = childObject.AddComponent<T>();
            childController.Setup(this);
            _children ??= new List<Controller>();
            _children.Add(childController);
            return childController;
        }

        public void Remove(Controller controller, bool destroy)
        {
            _children?.Remove(controller);
            if (destroy)
            {
                if (Application.isPlaying)
                {
                    Destroy(controller.gameObject);
                }
                else
                {
                    DestroyImmediate(controller.gameObject);
                }
            }
            RefreshLayout();
        }

        public bool Visibility
        {
            get => _visibility ;
            private set
            {
                if (_visibility != value)
                {
                    _visibility = value;
                    OnVisibilityChanged();
                }
            }
        }

        public void Hide()
        {
            Visibility = false;
        }

        public void Show()
        {
            Visibility = true;
        }

        public void ToggleVisibility()
        {
            Visibility = !GameObject.activeSelf;
        }

        public void OnVisibilityChanged()
        {
            GameObject.SetActive(Visibility);
            OnVisibilityChangedEvent?.Invoke(this);
        }

        private static Vector2 GetVec2FromLayout(TextAnchor anchor)
        {
            var index = (int)anchor;
            return new Vector2((index % 3) * 0.5f, 1.0f - (int)(index/3) * 0.5f);
        }

        public void RefreshLayout()
        {
            RefreshLayoutInternal();
            AdjustHeight();
        }

        public virtual void RefreshLayoutInternal()
        {
            if (RectTransform != null)
            {
                RectTransform.pivot = GetVec2FromLayout(_layoutStyle.pivot);
                RectTransform.anchorMin = GetVec2FromLayout(_layoutStyle.anchor);
                RectTransform.anchorMax = GetVec2FromLayout(_layoutStyle.anchor);

                switch (_layoutStyle.layout)
                {
                    case LayoutStyle.Layout.Fixed:
                        break;

                    case LayoutStyle.Layout.Fill:
                        RectTransform.anchorMin = new Vector2(0.0f, 0.0f);
                        RectTransform.anchorMax = new Vector2(1.0f, 1.0f);
                        break;
                    case LayoutStyle.Layout.FillHorizontal:
                        RectTransform.anchorMin = new Vector2(0.0f, RectTransform.anchorMin.y);
                        RectTransform.anchorMax = new Vector2(1.0f, RectTransform.anchorMax.y);
                        break;
                    case LayoutStyle.Layout.FillVertical:
                        RectTransform.anchorMin = new Vector2(RectTransform.anchorMin.x, 0.0f);
                        RectTransform.anchorMax = new Vector2(RectTransform.anchorMax.x, 1.0f);
                        break;
                }

                RectTransform.offsetMin = new Vector2(_layoutStyle.LeftMargin, _layoutStyle.BottomMargin);
                RectTransform.offsetMax = new Vector2(-_layoutStyle.RightMargin, -_layoutStyle.TopMargin);

                var fixedWidth = _layoutStyle.size.x != 0.0f ? _layoutStyle.size.x : RectTransform.sizeDelta.x;
                var fixedHeight = _layoutStyle.size.y != 0.0f ? _layoutStyle.size.y : RectTransform.sizeDelta.y;
                RectTransform.sizeDelta = new Vector2(fixedWidth, fixedHeight);

                if (_layoutStyle.masks)
                {
                    _mask ??= GameObject.AddComponent<RectMask2D>();
                    _mask.enabled = true;
                }
                else if(_mask != null)
                {
                    _mask.enabled = false;
                }
            }

            if (_children != null)
            {
                foreach (var child in _children)
                {
                    child.RefreshLayout();
                }
            }
        }

        protected virtual void AdjustHeight()
        {
            if (!LayoutStyle.adaptHeight) return;

            if (RectTransform == null) return;

            var maxOffset = 0.0f;

            if (_children != null)
            {
                foreach (var child in _children)
                {
                    if (child is Flex flex)
                    {
                        maxOffset = Mathf.Max(maxOffset, flex.SizeDeltaWithMargin.y);
                    }
                }
            }

            RectTransform.sizeDelta = new Vector2(RectTransform.sizeDelta.x, maxOffset);
        }

        public void OnDestroy()
        {
            if (Owner != null)
            {
                Owner.Remove(this, false);
            }
        }
    }
}

#endif // OVR_INTERNAL_CODE
