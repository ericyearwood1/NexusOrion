using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using ProtoKit.GraphicBase;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class HeadPoseCursorVisual : MonoBehaviour
    {
        [SerializeField, ReadOnly] private Selector _selector;
        [SerializeField, ReadOnly] private InputContext _context;
        [SerializeField, ReadOnly] private OCLayoutSize _cursorLayout;
        [SerializeField, ReadOnly] private RoundedRect _cursorRoundedRect;
        [SerializeField, ReadOnly] private Canvas _canvas;

        [SerializeField] private Vector3 _defaultSize = new Vector2(0.01f, 0.01f);
        [SerializeField] private float _defaultRadius = 16;
        [SerializeField] private float _hoveredRadius = 8;
        [SerializeField] private float _hoveredPadding = 0;
        [SerializeField] private float _border = 1;
        [SerializeField] private float _cursorSurfaceOffset = -0.01f;

        [SerializeField]
        private OneEuroFilterPropertyBlock _positionFilterProperties =
            new OneEuroFilterPropertyBlock(2f, 3f);

        private IOneEuroFilter<Vector3> _positionFilter;

        [SerializeField, ReadOnly] private Target _hovered;
        [SerializeField, ReadOnly] private float _hoveredLerp;

        [SerializeField, ReadOnly] private Vector3 _hoveredPosition;
        [SerializeField, ReadOnly] private Vector3 _hoveredSize;
        [SerializeField, ReadOnly] private Vector3 _oneEuroPosition;

        private void Awake()
        {
            this.Scene(ref _selector);
            this.Sibling(ref _context);

            _canvas = this.CreateChild<Canvas>(name: "Cursor");

            _positionFilter = OneEuroFilter.CreateVector3();

            _canvas.renderMode = RenderMode.WorldSpace;
            _canvas.additionalShaderChannels |=
                AdditionalCanvasShaderChannels.TexCoord1 |
                AdditionalCanvasShaderChannels.TexCoord2 |
                AdditionalCanvasShaderChannels.TexCoord3;

            _cursorRoundedRect = _canvas.CreateChild<RoundedRect>();

            _cursorRoundedRect.SetMaterial(Resources.Load("ARGlassesCursorVisual") as Material);
            _cursorRoundedRect.SetColorA(new Color(1, 1, 1, 0.5f));
            _cursorRoundedRect.SetBorder(_border);
            _cursorRoundedRect.SetRadius(_defaultRadius);
            _cursorRoundedRect.SetBorderColor(new Color(0, 0, 0, 0.5f));

            _cursorLayout = _cursorRoundedRect.Ensure<OCLayoutSize>();
            _cursorLayout.SetSize2D(_defaultSize);
        }

        private void OnEnable()
        {
            _canvas.gameObject.SetActive(true);
        }

        private void OnDisable()
        {
            _canvas.gameObject.SetActive(false);
        }

        private void LateUpdate()
        {
            if (_selector.Selected || !_context || !_context.TargetContext)
            {
                _cursorRoundedRect.enabled = false;
                return;
            }

            _cursorRoundedRect.enabled = true;

            // _hovered = _controller.Hovered;
            // _hoveredLerp = Mathf.Lerp(_hoveredLerp, _hovered ? 1 : 0, Time.deltaTime * 8);
            // if (_hovered)
            // {
            //     _hoveredPosition = _hovered.transform.position;
            //     _hoveredSize = _defaultSize;

            //     if (_hovered.TryGetComponent<OCLayoutComponentBase>(out var layout)) _hoveredSize = layout.CurrentSize;
            //     if (_hovered.TryGetComponent<RectTransform>(out var rectTransform)) _hoveredSize = rectTransform.rect.size;
            // }

            var ctx = _cursorLayout.UnitsContext;
            var size = Vector3.Lerp(_defaultSize, _hoveredSize, _hoveredLerp);
            _cursorLayout.SetWidth(size.x.AsMeters() + _hoveredPadding.AsPixels().ToMeters(ctx));
            _cursorLayout.SetHeight(size.y.AsMeters() + _hoveredPadding.AsPixels().ToMeters(ctx));
            _cursorRoundedRect.SetRadius(Mathf.Lerp(_defaultRadius, _hoveredRadius, _hoveredLerp));

            _positionFilter.SetProperties(_positionFilterProperties);
            _oneEuroPosition = _positionFilter.Step(_context.CursorWorld, Time.deltaTime);

            _cursorLayout.transform.position = Vector3.Lerp(_oneEuroPosition, _hoveredPosition, _hoveredLerp);
            _cursorLayout.transform.position += _context.TargetContext.Forward * _cursorSurfaceOffset;

            _cursorLayout.transform.rotation = Quaternion.LookRotation(_context.TargetContext.Forward);
        }
    }
}
