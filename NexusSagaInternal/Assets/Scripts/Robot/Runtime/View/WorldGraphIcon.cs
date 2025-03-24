using Robot.Runtime.Data.WorldGraph;
using Robot.Runtime.Utils;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Robot.Runtime.View
{
    public class WorldGraphIcon : SimpleAnimatable
    {
        [SerializeField] private CanvasGroup _canvasGroup;
        [SerializeField] private RectTransform _mainTransform;
        [SerializeField] private RectTransform _iconTransform;
        [SerializeField] private Image _icon;
        [SerializeField] private TMP_Text _label;
        private WorldGraphNodeData _node;

        public WorldGraphNodeData Node => _node;

        public void Initialise(WorldGraphNodeData node, Sprite icon, Color color, string label = null)
        {
            _node = node;
            var iconSize = icon.bounds.size;
            var ratio = iconSize.x / iconSize.y;
            var mainRect = _mainTransform.rect;
            var imageRect = _iconTransform.rect;
            var width = ratio * imageRect.height;
            _iconTransform.sizeDelta = new Vector2(width, imageRect.height);
            _mainTransform.sizeDelta = new Vector2(width, mainRect.height);
            _icon.sprite = icon;
            _icon.color = color;
            _label.text = label ?? string.Empty;
        }

        protected override void UpdateDisplay(float progress)
        {
            _canvasGroup.alpha = progress;
        }

        public override void Reset()
        {
            base.Reset();
            _node = null;
            _icon.sprite = null;
            if (_label == null) return;
            _label.text = string.Empty;
        }
    }
}