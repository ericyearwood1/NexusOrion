using Robot.Runtime.Data.WorldGraph;
using UnityEngine;

namespace Robot.Runtime.View
{
    public class WorldGraphObjectHighlight : TargetObjectHighlightView, IResettable
    {
        private WorldGraphNodeData _data;
        public bool IsDisposed { get; set; }
        public WorldGraphNodeData Data => _data;

        public void Initialise(WorldGraphNodeData data, Transform cameraTransform, Color color)
        {
            _color = color;
            _data = data;
            gameObject.name = $"ObjectHighlight_{data.Name}";
            transform.localPosition = data.Position;
            transform.localEulerAngles = new Vector3(0, data.Yaw, 0);
            _cameraTransform = cameraTransform;
            _label.text = data.Name;
            UpdateDisplay(0);
        }

        public void Show()
        {
            if (IsDisposed) return;
            StartShow(_cameraTransform);
        }

        public void Reset()
        {
            IsDisposed = false;
            _data = null;
        }

        public void ShowImmediate()
        {
            UpdateDisplay(1);
        }
        public void Popup()
        {
            // Code to show the highlight as a popup
            Show();
            Invoke(nameof(HideAfterDelay), 2f);
        }
        private void HideAfterDelay()
        {
            // Code to hide the highlight after a delay
            Hide();
        }
    }
}