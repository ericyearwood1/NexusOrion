using System;
using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using ProtoKit.GraphicBase;
using UnityEngine;
using System.Collections;
using ARGlasses.Interaction.Motion;

namespace ARGlasses.Interaction
{
    public class MagneticGlowCursorVisual : MonoBehaviour
    {
        [SerializeField, ReadOnly] private Selector _selector;
        [SerializeField, ReadOnly] private OCLayoutSize _cursorLayout;
        [SerializeField, ReadOnly] private RoundedRect _cursorRoundedRect;
        [SerializeField, ReadOnly] private Canvas _canvas;
        [SerializeField, ReadOnly] private Material _cursorMat;

        [SerializeField] private Vector3 _defaultSize = new Vector2(0.01f, 0.01f);
        [SerializeField] private float _glowSize = 0.05f;
        [SerializeField] private float _defaultRadiusPixels = 40;
        [SerializeField] private float _hoveredRadiusPixels = 40;
        [SerializeField] private float _border = 0.75f;
        [SerializeField] private float _cursorSurfaceOffset = 0f;
        [SerializeField] private float _glowStrengthPow = 2f;

        [SerializeField] private float _selectSizeScale = 0.5f;
        [SerializeField] private float _maxCursorLerpSize = 3f;
        [SerializeField] private float _maxCursorLerpDistance = 0.03f;

        [SerializeField, ReadOnly] private Pose _lastHoveredTargetPose = Pose.identity;
        [SerializeField, ReadOnly] private Vector3 _lastHoveredTargetSize;

        [SerializeField, ReadOnly] private CursorEffectStyle _hoveredTargetEffectStyle;

        private ICursorEffect _hoveredTargetEffect;

        private float _cursorLerpDuration = 0.10f;
        private float _backgroundLerpDuration = 0.075f;

        private float _cursorSmallScale = 2.5f;
        private float _cursorCurrentScale;
        private float _backgroundSmallScale = 0.9f;
        private float _backgroundCurrentScale;

        private static readonly int GlowOffsetMetersID = Shader.PropertyToID("_CursorPosition");
        private static readonly int SpecularStrengthID = Shader.PropertyToID("_SpecularStrength");
        private static readonly int SpecularRadiusID = Shader.PropertyToID("_SpecularRadius");

        [SerializeField] private MotionParams _motionParams = new(1.5f, 0.8f, 0.5f);
        [SerializeField] private Motion<MotionState> _motion;
        [SerializeField, ReadOnly] private MotionState _goal;
        [SerializeField] private float _sizeBasedLerpScale = 0.5f;
        [SerializeField] private float _blur = 12.5f;
        [SerializeField, ReadOnly] private MotionState _motionState;

        [Serializable]
        struct MotionState
        {
            [SerializeField] public Vector3 Size;
            [SerializeField] public Vector3 Position;
            [SerializeField] public Quaternion Rotation;
            [SerializeField] public float GlowStrength;

            public MotionState(Vector3 cursorSize)
            {
                Size = cursorSize;
                Position = default;
                Rotation = Quaternion.identity;
                GlowStrength = 0;
            }
        }

        [SerializeField, ReadOnly] private RevealStyle _reveal;
        private Vector3 _surfaceChangeHidePosition;
        [SerializeField] private float _surfaceChangeUnhideMagnitude = 0.05f;
        private float _glowRadius;
        private float _specularStrength;

        private void Awake()
        {
            this.Scene(ref _selector);
            _canvas = this.CreateChild<Canvas>(name: "MagneticGlowCursor");
            _canvas.renderMode = RenderMode.WorldSpace;
            _canvas.sortingOrder = 1;
            _canvas.additionalShaderChannels |= AdditionalCanvasShaderChannels.TexCoord1 | AdditionalCanvasShaderChannels.TexCoord2 | AdditionalCanvasShaderChannels.TexCoord3;
            // _canvas.transform.SetParent(Camera.main.transform, worldPositionStays: false);
            // _canvas.transform.localPosition = new Vector3(0, 0, 0.35f);
            _canvas.transform.SetRectSize(0.2f, 0.2f);
            _canvas.Ensure<ArtemisUnitSettings>();

            _cursorRoundedRect = _canvas.CreateChild<RoundedRect>();
            _cursorMat = new Material(Resources.Load("MagneticGlowCursor", typeof(Material)) as Material);
            _cursorRoundedRect.SetMaterial(_cursorMat);
            _cursorRoundedRect.SetColorA(Color.white);
            _cursorRoundedRect.SetRadius(_defaultRadiusPixels);
            _cursorRoundedRect.SetBorder(_border);
            _cursorRoundedRect.SetBorderColor(Color.black);
            _cursorRoundedRect.SetBorderType(3);
            _cursorRoundedRect.SetGlowColor(Color.clear);
            _cursorRoundedRect.materialForRendering.SetFloat("_HideOnBlack", 0.5f);
            _cursorLayout = _cursorRoundedRect.Ensure<OCLayoutSize>();
        }

        private void OnEnable()
        {
            _canvas.enabled = true;
            _selector.WhenSelecting += HandleSelecting;
            _selector.WhenHovering += HandleHovering;
            _selector.WhenNoFocus += HandleNoFocus;
            _canvas.gameObject.SetActive(true);
            _motion = new Motion<MotionState>(new MotionState(_defaultSize), _motionParams);
            InitMotion(RevealStyle.Show);
        }

        private void OnDisable()
        {
            _canvas.enabled = false;
            _selector.WhenSelecting -= HandleSelecting;
            _selector.WhenHovering -= HandleHovering;
            _selector.WhenNoFocus -= HandleNoFocus;
            _canvas.gameObject.SetActive(false);
        }

        private void HandleNoFocus(InteractionStateSequence state)
        {
            ClearTargetEffect();
        }

        private void ClearTargetEffect()
        {
            if (_hoveredTargetEffect != null) _hoveredTargetEffect.SetBackgroundOffset(Vector3.zero);
            _hoveredTargetEffect = null;
            _hoveredTargetEffectStyle = null;
        }

        private void HandleHovering(IHover hover)
        {
            Target target = hover.Target;

            var newTargetEffect = target.GetComponentInParent<ICursorEffect>();
            if (newTargetEffect != _hoveredTargetEffect) ClearTargetEffect();

            _hoveredTargetEffect = newTargetEffect;
            _hoveredTargetEffectStyle = target.GetComponentInParent<CursorEffectStyle>();

            _lastHoveredTargetSize = target.CurrentSize;
            _lastHoveredTargetPose = target.transform.ToPose();

            if (_hoveredTargetEffect != null)
            {
                Vector3 offset = _selector.CursorWorld - target.transform.position;
                _lastHoveredTargetPose.position += Vector3.ClampMagnitude(offset, _hoveredTargetEffect.NudgeClamp);
                _hoveredTargetEffect.SetBackgroundOffset(target.transform.InverseTransformPoint(_lastHoveredTargetPose.position));
            }
        }

        private void HandleSelecting(ISelection selection)
        {
        }

        public void CursorSurfaceChanged(RevealStyle revealStyle)
        {
            InitMotion(revealStyle);
        }

        public enum RevealStyle
        {
            None,
            Show,
            Hide,
            RippleIn,
        }

        private void InitMotion(RevealStyle revealStyle)
        {
            _reveal = revealStyle;
            _surfaceChangeHidePosition = _selector.CursorWorld;
            var goal = CreateGoalMotion();
            _motion.SetCurrent(goal);
            _motion.SetGoal(goal);
            ApplyCurrentMotion();
        }

        private void Update()
        {
            var goal = CreateGoalMotion();
            _motionState = _motion.Step(goal, Time.deltaTime);
            ApplyCurrentMotion();
        }

        private MotionState CreateGoalMotion()
        {
            MotionState goal = default;
            var surface = _selector.TargetContext;
            var cursorWorld = _selector.CursorWorld;
            if (!surface || cursorWorld == default)
            {
                _canvas.enabled = false;
                return _motion.Goal;
            }

            _canvas.enabled = true;

            var isGlowHover = _hoveredTargetEffect != null && (_hoveredTargetEffectStyle == null || _hoveredTargetEffectStyle.SelectedMode != CursorEffectStyle.Mode.ShowPointer);
            var isPinching = _selector.IsPinching(Side.Right, HandFinger.Index);

            if (_reveal == RevealStyle.Hide)
            {
                var movementThresholdMet = (_selector.CursorWorld - _surfaceChangeHidePosition).magnitude > _surfaceChangeUnhideMagnitude;
                if (isPinching || movementThresholdMet) _reveal = RevealStyle.None;
            }

            goal.GlowStrength = isGlowHover || isPinching || _reveal == RevealStyle.Hide ? 1f : 0f;

            if (isGlowHover) goal.Size = _lastHoveredTargetSize;
            else
            {
                goal.Size = _defaultSize;
                if (isPinching) goal.Size *= _selectSizeScale;
            }

            goal.Position = (isGlowHover ? _lastHoveredTargetPose.position : cursorWorld) + (surface.Forward * _cursorSurfaceOffset);
            goal.Rotation = isGlowHover ? _lastHoveredTargetPose.rotation : Quaternion.LookRotation(surface.Forward);

            return _goal = goal;
        }

        private void ApplyCurrentMotion()
        {
            MotionState currentMotion = _motion.Current;
            Vector3 cursorWorld = _selector.CursorWorld;
            _cursorLayout.SetWidth(currentMotion.Size.x.AsMeters());
            _cursorLayout.SetHeight(currentMotion.Size.y.AsMeters());

            var steppedMotionPosition = Vector3.Lerp(cursorWorld, currentMotion.Position, currentMotion.GlowStrength);
            _cursorLayout.transform.SetPositionAndRotation(steppedMotionPosition, currentMotion.Rotation);

            var hideLargeSize = Mathf.Clamp01(currentMotion.Size.MaxComponent() / _defaultSize.MaxComponent() * _sizeBasedLerpScale - 1);
            var sizeBasedGlowStrength = Mathf.Clamp01(hideLargeSize + currentMotion.GlowStrength);

            _cursorRoundedRect.enabled = _reveal != RevealStyle.Hide;

            _cursorRoundedRect.SetColorA(Color.Lerp(Color.white, Color.white.WithAlpha(0.01f), sizeBasedGlowStrength));
            _cursorRoundedRect.SetBorderColor(Color.Lerp(Color.black.WithAlpha(1f), Color.white.WithAlpha(0), sizeBasedGlowStrength));
            _cursorRoundedRect.SetBorder(Mathf.Lerp(_border, 0, currentMotion.GlowStrength));
            _cursorRoundedRect.SetGlowSpread(Mathf.Lerp(-20, -6, currentMotion.GlowStrength));
            _cursorRoundedRect.SetGlowBlur(_blur);

            float radiusPixels = _hoveredTargetEffect != null ? _hoveredTargetEffect.GetRadiusPixels() : _hoveredRadiusPixels;
            _cursorRoundedRect.SetRadius(Mathf.Lerp(_defaultRadiusPixels, radiusPixels, currentMotion.GlowStrength));

            var size = currentMotion.Size;
            var cursorGlowLocalPosition = _cursorRoundedRect.transform.InverseTransformPoint(cursorWorld);
            Vector2 glowOffsetScale = new Vector2(size.x / Mathf.Max(size.x, size.y), size.y / Mathf.Max(size.x, size.y));
            var goalOffsetMeters = new Vector2(
                Mathf.Lerp(0.5f - glowOffsetScale.x * 0.5f, 0.5f + glowOffsetScale.x * 0.5f, cursorGlowLocalPosition.x / size.x + 0.5f),
                Mathf.Lerp(0.5f - glowOffsetScale.y * 0.5f, 0.5f + glowOffsetScale.y * 0.5f, cursorGlowLocalPosition.y / size.y + 0.5f)
            );
            _cursorRoundedRect.materialForRendering.SetVector(GlowOffsetMetersID, goalOffsetMeters);

            var hideGlow = _hoveredTargetEffectStyle != null && _hoveredTargetEffectStyle.SelectedMode == CursorEffectStyle.Mode.HideGlow;
            _specularStrength = hideGlow ? 0 : currentMotion.GlowStrength * 0.5f;
            _cursorRoundedRect.materialForRendering.SetFloat(SpecularStrengthID, _specularStrength);

            _glowRadius = _glowSize / Mathf.Max(size.x, size.y);
            _cursorRoundedRect.materialForRendering.SetFloat(SpecularRadiusID, currentMotion.GlowStrength * _glowRadius);
        }
    }
}
