using System.Collections.Generic;
using System.Linq;
using Robot.Runtime.Data.WorldGraph;
using Robot.Runtime.Utils;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using HighlightState = Robot.Runtime.Data.HighlightState;

namespace Robot.Runtime.View
{
    public class FurnitureHighlight : SimpleAnimatable
    {
        private static readonly int Color1 = Shader.PropertyToID("_BaseColor");
        [SerializeField] private Transform _furnitureExtentsView;
        [SerializeField] private Transform _offsetTransform;
        [SerializeField] private MeshRenderer _meshRenderer;
        [SerializeField] private CanvasGroup _group;
        [SerializeField] private Transform _list;
        [SerializeField] private TMP_Text _label;
        [SerializeField] private Image _labelBackground;
        private WorldGraphNodeData _data;
        private MaterialPropertyBlock _propertyBlock;
        private List<WorldGraphIcon> _activeIcons = new();
        private List<Vector3> _orderedPositionList = new();
        private Color _color;

        private Transform _cameraTransform;
        private HighlightPool<WorldGraphIcon> _pool;
        private bool _isAlwaysHide;
        private Color _defaultColor;

        public WorldGraphNodeData Data => _data;

        public void Initialise(Transform cameraTransform, WorldGraphNodeData data, HighlightPool<WorldGraphIcon> pool,
            bool isShowFlatExtends, bool isUseObjectIdForFurnitureLabel, Color color)
        {
            _defaultColor = color;
            _data = data;
            _pool = pool;
            var label = isUseObjectIdForFurnitureLabel ? data.Id : data.Name;
            _label.text = label.Replace("_"," ");
            _cameraTransform = cameraTransform;
            gameObject.name = $"FurnitureHighlight_{data.Id}";
            _list.gameObject.SetActive(false);
            SetPositionAndScale(data, isShowFlatExtends);
            AnchorInfoPanelToBackLeftCorner(data);
            InitialiseMaterialForAnimation(color);
            UpdateDisplay(0);
        }

        private void InitialiseMaterialForAnimation(Color color)
        {
            _color = new Color(color.r, color.g, color.b, 0);
            _labelBackground.color = color;
            _propertyBlock = new MaterialPropertyBlock();
            _propertyBlock.SetColor(Color1, _color);
            _meshRenderer.SetPropertyBlock(_propertyBlock);

            if (_data.Name.ToLower().Contains("floor"))
            {
                _isAlwaysHide = true;
            }
        }

        private void SetPositionAndScale(WorldGraphNodeData data, bool isShowFlatExtends)
        {
            if (isShowFlatExtends)
            {
                var position = data.Position;

                var scale = data.Extents;//* 2;
                position.y = scale.y - 0.0005f;
                scale.y = 0.001f;
                _furnitureExtentsView.localScale = scale;
                transform.localPosition = position;
            }
            else
            {
                _furnitureExtentsView.localScale = data.Extents;
                transform.localPosition = data.Position;
            }
            
            transform.localEulerAngles = new Vector3(0, data.Yaw, 0);
        }

        private void AnchorInfoPanelToBackLeftCorner(WorldGraphNodeData data)
        {
            var localScale = _furnitureExtentsView.localScale;
            
            var backLeftCorner = _furnitureExtentsView.localPosition;
            backLeftCorner.x -= localScale.x * 0.5f;
            backLeftCorner.y += localScale.y * 0.5f;// + 0.1f;
            backLeftCorner.z += localScale.z * 0.5f;

            var backRightCorner = backLeftCorner;
            backRightCorner.x += localScale.x;
            
            var frontLeftCorner = backLeftCorner;
            frontLeftCorner.z -= localScale.z;
            
            var frontRightCorner = frontLeftCorner;
            frontRightCorner.x += localScale.x;
            var parent = transform.parent;
            var backLeft = parent.InverseTransformPoint(transform.TransformPoint(backLeftCorner));
            var backRight = parent.InverseTransformPoint(transform.TransformPoint(backRightCorner));
            var frontLeft = parent.InverseTransformPoint(transform.TransformPoint(frontLeftCorner));
            var frontRight = parent.InverseTransformPoint(transform.TransformPoint(frontRightCorner));

            _orderedPositionList.Clear();
            _orderedPositionList.Add(backLeft);
            _orderedPositionList.Add(backRight);
            _orderedPositionList.Add(frontLeft);
            _orderedPositionList.Add(frontRight);
            var orderedList = _orderedPositionList.OrderBy(o=>o.x).ToList();
            var backCornerA = orderedList[0];
            var backCornerB = orderedList[1];
            var anchorPoint = backCornerA.z > backCornerB.z ? backCornerA : backCornerB;
            anchorPoint.y += 0.1f;
            _offsetTransform.position = parent.TransformPoint(anchorPoint); 
        }

        private void Billboard()
        {
            if (_cameraTransform == null)
            {
                return;
            }
            Vector3 forward = _offsetTransform.position - _cameraTransform.position;
            forward.y = 0;
            _offsetTransform.rotation =  Quaternion.LookRotation(forward);
        }

        protected override void Update()
        {
            base.Update();
            CheckIconForRemoval();
        }

        private void LateUpdate()
        {
            Billboard();
        }

        private void CheckIconForRemoval()
        {
            for (var i = _activeIcons.Count - 1; i >= 0; --i)
            {
                var icon = _activeIcons[i];
                if(RemoveHighlightIfAnimatedOut(icon, i)) continue;
                if (icon.Node == null || icon.Node.IsRemoved)
                {
                    RemoveImmediatelyAt(i, icon);
                    // if (icon.State is (HighlightState.OnDisplay or HighlightState.AnimatingIn))
                    // {
                    //     Debug.Log($"FurnitureHighlight::{_data.Id}::CheckIconForRemoval::Hide {icon}");
                    //     icon.Hide();
                    // }
                    // else
                    // {
                    //     RemoveImmediatelyAt(i, icon);
                    // }
                }
            }
        }

        private bool RemoveHighlightIfAnimatedOut(WorldGraphIcon icon, int i)
        {
            if (!icon.IsDisposed && (icon.Node != null && !icon.Node.IsRemoved)) return false;
            if (icon.State is not (HighlightState.Hidden or HighlightState.None)) return false;
            RemoveImmediatelyAt(i, icon);
            return true;
        }

        private void RemoveImmediatelyAt(int i, WorldGraphIcon icon)
        {
            _activeIcons.RemoveAt(i);
            _pool.Return(icon);
            if (_activeIcons.Count == 0)
            {
                _list.gameObject.SetActive(false);
            }
        }

        public override void Reset()
        {
            base.Reset();
            _data = null;
            UpdateDisplay(0);
            gameObject.SetActive(false);
            _list.gameObject.SetActive(false);
            for (var i = _activeIcons.Count - 1; i >= 0; --i)
            {
                var icon = _activeIcons[i];
                RemoveImmediatelyAt(i, icon);
            }
        }

        public WorldGraphIcon AddObject(WorldGraphNodeData data, Sprite icon)
        {
            var iconView = _pool.Get();
            var transform = iconView.transform;
            transform.parent = _list;
            transform.localScale = Vector3.one;
            transform.localPosition = Vector3.zero;
            transform.localRotation = Quaternion.identity;
            iconView.Initialise(data, icon, _defaultColor, data.Name);
            iconView.ShowImmediate();
            _activeIcons.Add(iconView);
            _list.gameObject.SetActive(true);
            return iconView;
        }

        public void RemoveObject(WorldGraphNodeData data)
        {
            for (var i = _activeIcons.Count - 1; i >= 0; --i)
            {
                var icon = _activeIcons[i];
                if (icon.Node.Id != data.Id) continue;
                RemoveImmediatelyAt(i, icon);
                // if (icon.State is HighlightState.Hidden or HighlightState.None)
                // {
                //     Debug.Log($"FurnitureHighlight::{_data.Id}::RemoveObject::RemoveImmediatelyAt {data.Id} | {icon.State}");
                //     RemoveImmediatelyAt(i, icon);
                // }
                // else
                // {
                //     Debug.Log($"FurnitureHighlight::{_data.Id}::RemoveObject::Hide {data.Id} | {icon.State}");
                //     icon.Hide();
                // }
            }
        }

        protected override void UpdateDisplay(float progress)
        {
            _color.a = progress;
            if (!_isAlwaysHide)
            {    
                _propertyBlock.SetColor(Color1, _color);
                _meshRenderer.SetPropertyBlock(_propertyBlock);
            }
            _group.alpha = progress;
        }
    }
}