using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using ProtoKit.GraphicBase;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class FreeMovingCursorVisual : MonoBehaviour
    {
        [SerializeField, ReadOnly] private Selector _selector;

        [SerializeField, ReadOnly] private OCLayoutSize _cursorLayout;
        [SerializeField, ReadOnly] private RoundedRect _cursorRoundedRect;
        [SerializeField, ReadOnly] private Canvas _canvas;

        [SerializeField] private Vector3 _defaultSize = new Vector2(0.02f, 0.02f);
        [SerializeField] private float _defaultRadius = 8;
        [SerializeField] private float _hoveredRadius = 8;
        [SerializeField] private float _border = 1;
        [SerializeField] private float _cursorSurfaceOffset = 0;
        [SerializeField] private float _hoverStrengthLerp = 8;

        [SerializeField, ReadOnly] private Vector2 _glowOffsetMeters;
        [SerializeField, ReadOnly] private float _hoverStrength;
        [SerializeField, ReadOnly] private Vector3 _hoveredSize;

        private void Awake()
        {
            this.Scene(ref _selector);
            _canvas = this.CreateChild<Canvas>(name: "FreeMovingCursor");
            _canvas.renderMode = RenderMode.WorldSpace;
            _canvas.sortingOrder = 1;
            _canvas.additionalShaderChannels |=
                AdditionalCanvasShaderChannels.TexCoord1 |
                AdditionalCanvasShaderChannels.TexCoord2 |
                AdditionalCanvasShaderChannels.TexCoord3;
            _cursorRoundedRect = _canvas.CreateChild<RoundedRect>();
            // _cursorRoundedRect.SetMaterial(Resources.Load("ARGlassesCursorVisual") as Material);
            _cursorRoundedRect.SetColorA(Color.white.WithAlpha(0.7f));
            _cursorRoundedRect.SetRadius(_defaultRadius);
            _cursorRoundedRect.SetBorder(_border);
            _cursorRoundedRect.SetBorderColor(Color.black.WithAlpha(0.9f));
            _cursorLayout = _cursorRoundedRect.Ensure<OCLayoutSize>();
            _cursorLayout.SetSize2D(_defaultSize);
        }

        private void OnEnable()
        {
            _canvas.enabled = true;
            _selector.WhenSelecting += HandleSelecting;
            _selector.WhenHovering += HandleHovering;
            _selector.WhenNoFocus += HandleNoFocus;
            _canvas.gameObject.SetActive(true);
            _hoverStrength = 0;
        }

        private void OnDisable()
        {
            _canvas.enabled = false;
            _selector.WhenSelecting -= HandleSelecting;
            _selector.WhenHovering -= HandleHovering;
            _selector.WhenNoFocus -= HandleNoFocus;
            _canvas.gameObject.SetActive(false);
        }

        private void HandleSelecting(ISelection selection)
        {
            LerpHoverStrength(true);
            _canvas.enabled = false;
        }

        private void HandleHovering(IHover hover)
        {
            LerpHoverStrength(false);
            UpdateVisual();
        }

        private void LerpHoverStrength(bool isHovered)
        {
            _hoverStrength = Mathf.Lerp(_hoverStrength, isHovered ? 1 : 0, Time.deltaTime * _hoverStrengthLerp);
        }

        private void HandleNoFocus(InteractionStateSequence state)
        {
            LerpHoverStrength(false);
            UpdateVisual();
        }

        private void UpdateVisual()
        {
            var surface = _selector.TargetContext;
            var cursorWorld = _selector.CursorWorld;
            if (!surface || cursorWorld == default)
            {
                _canvas.enabled = false;
                return;
            }
            var surfaceForward = surface.Forward;

            _canvas.enabled = true;
            var ctx = _cursorLayout.UnitsContext;
            var size = Vector3.Lerp(_defaultSize, _hoveredSize, _hoverStrength);
            _cursorLayout.SetWidth(size.x.AsMeters());
            _cursorLayout.SetHeight(size.y.AsMeters());
            _cursorRoundedRect.SetRadius(Mathf.Lerp(_defaultRadius, _hoveredRadius, _hoverStrength));

            var pos = cursorWorld;
            pos += surfaceForward * _cursorSurfaceOffset;
            var rot = Quaternion.LookRotation(surfaceForward);
            _cursorLayout.transform.SetPositionAndRotation(pos, rot);
        }

        public void CursorSurfaceChanged()
        {
        }
    }
}
