using UnityEngine;

namespace ARGlasses.Interaction
{
    public class CursorVisual : MonoBehaviour
    {
        public enum Mode
        {
            None,
            MagneticGlow,
            FreeMoving,
            MAX
        }

        [SerializeField] private Mode _selectedMode = Mode.MagneticGlow;
        public Mode VisualMode
        {
            get => _selectedMode;
            set
            {
                _selectedMode = value;
                _freeMovingCursorVisual.enabled = _selectedMode == Mode.FreeMoving && isActiveAndEnabled;
                _magneticGlowCursorVisual.enabled = _selectedMode == Mode.MagneticGlow && isActiveAndEnabled;
            }
        }

        public void CycleMode() => VisualMode = (Mode)(((int)_selectedMode + 1) % (int)Mode.MAX);

        [SerializeField, ReadOnly] FreeMovingCursorVisual _freeMovingCursorVisual;
        [SerializeField, ReadOnly] MagneticGlowCursorVisual _magneticGlowCursorVisual;
        void Awake()
        {
            this.CreateChild(ref _freeMovingCursorVisual);
            this.CreateChild(ref _magneticGlowCursorVisual);
        }

        private void OnEnable()
        {
            VisualMode = _selectedMode;
        }

        private void OnDisable()
        {
            _freeMovingCursorVisual.enabled = false;
            _magneticGlowCursorVisual.enabled = false;
        }

        public void CursorSurfaceChanged(MagneticGlowCursorVisual.RevealStyle revealStyle)
        {
            _freeMovingCursorVisual.CursorSurfaceChanged();
            _magneticGlowCursorVisual.CursorSurfaceChanged(revealStyle);
        }
    }
}
