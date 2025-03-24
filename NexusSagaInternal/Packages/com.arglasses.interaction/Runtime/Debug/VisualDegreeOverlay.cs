using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class VisualDegreeOverlay : MonoBehaviour
    {
        [SerializeField] private float _distanceLerp = 8f;
        [SerializeField] private bool _showGrid;
        [SerializeField] private bool _showFov;

        [SerializeField, ReadOnly] private Selector _selector;
        [SerializeField, ReadOnly] private TargetContext _targetContext;
        [SerializeField, ReadOnly] private InputContext _inputContext;

        [SerializeField, ReadOnly] private VisualDegreeOverlayRing[] _rings;
        [SerializeField, ReadOnly] private Color _defaultLineColor;
        [SerializeField, ReadOnly] private Color _defaultFovColor;
        [SerializeField, ReadOnly] private Renderer _sphereRenderer;
        [SerializeField, ReadOnly] private Material _material;
        [SerializeField, ReadOnly] private Conecaster _conecaster;

        private static readonly int LineIncrements = Shader.PropertyToID("_LineIncrements");
        private static readonly int LineWidths = Shader.PropertyToID("_LineWidths");
        private static readonly int LineColor = Shader.PropertyToID("_LineColor");
        private static readonly int Fov = Shader.PropertyToID("_Fov");
        private static readonly int FovColor = Shader.PropertyToID("_FovColor");

        private static readonly int FillColor0 = Shader.PropertyToID("_FillColor0");
        private static readonly int BorderColor0 = Shader.PropertyToID("_BorderColor0");
        private static readonly int Radius0 = Shader.PropertyToID("_Radius0");
        private static readonly int BorderWidth0 = Shader.PropertyToID("_BorderWidth0");
        private static readonly int Position0 = Shader.PropertyToID("_Position0");

        private static readonly int FillColor1 = Shader.PropertyToID("_FillColor1");
        private static readonly int BorderColor1 = Shader.PropertyToID("_BorderColor1");
        private static readonly int Radius1 = Shader.PropertyToID("_Radius1");
        private static readonly int BorderWidth1 = Shader.PropertyToID("_BorderWidth1");
        private static readonly int Position1 = Shader.PropertyToID("_Position1");

        private static readonly int FillColor2 = Shader.PropertyToID("_FillColor2");
        private static readonly int BorderColor2 = Shader.PropertyToID("_BorderColor2");
        private static readonly int Radius2 = Shader.PropertyToID("_Radius2");
        private static readonly int BorderWidth2 = Shader.PropertyToID("_BorderWidth2");
        private static readonly int Position2 = Shader.PropertyToID("_Position2");

        private static int RingPositionId(int ringIndex) =>
            ringIndex switch
            {
                0 => Position0,
                1 => Position1,
                2 => Position2,
            };

        private static int RingRadiusId(int ringIndex) =>
            ringIndex switch
            {
                0 => Radius0,
                1 => Radius1,
                2 => Radius2,
            };

        private static int RingFillColorId(int ringIndex) =>
            ringIndex switch
            {
                0 => FillColor0,
                1 => FillColor1,
                2 => FillColor2,
            };

        private static int RingBorderColorId(int ringIndex) =>
            ringIndex switch
            {
                0 => BorderColor0,
                1 => BorderColor1,
                2 => BorderColor2,
            };

        private static int RingBorderWidthId(int ringIndex) =>
            ringIndex switch
            {
                0 => BorderWidth0,
                1 => BorderWidth1,
                2 => BorderWidth2,
            };

        public bool ShowGrid
        {
            get => _showGrid;
            set
            {
                _showGrid = value;
                _material.SetColor(LineColor, _showGrid ? _defaultLineColor : new Color(0, 0, 0, 0));
            }
        }

        public bool ShowFov
        {
            get => _showFov;
            set
            {
                _showFov = value;
                _material.SetColor(FovColor, _showFov ? _defaultFovColor : new Color(0, 0, 0, 0));
            }
        }

        private void Awake()
        {
            this.Scene(ref _selector);
            _material = ExtensionsUnity.LoadMaterial("VisualDegreeOverlay");

            _sphereRenderer = ExtensionsUnity.CreateSphereRenderer(transform, name: "VisualDegreeOverlaySphere");
            _sphereRenderer.material = _material;

            _defaultLineColor = _material.GetColor(LineColor);
            _defaultFovColor = _material.GetColor(FovColor);

            _rings = new[]
            {
                this.CreateChild<VisualDegreeOverlayRing>(name: "Ring0"),
                this.CreateChild<VisualDegreeOverlayRing>(name: "Ring1"),
                this.CreateChild<VisualDegreeOverlayRing>(name: "Ring2"),
            };
            _rings[0].Init(Color.red.WithAlpha(0.04f), 4.5f, 0.2f);
            _rings[1].Init(Color.green.WithAlpha(0.04f), 1.5f, 0.2f);
            _rings[2].Init(Color.white.WithAlpha(0.04f), 0.15f, 0f);
        }

        private void OnEnable()
        {
            foreach (var ring in _rings) ring.ShowRing = false;
            ShowFov = ShowFov;
            ShowGrid = ShowGrid;
            _selector.WhenContextChanged += HandleContextChanged;
            _sphereRenderer.enabled = true;
        }

        private void OnDisable()
        {
            _selector.WhenContextChanged -= HandleContextChanged;
            _sphereRenderer.enabled = false;
        }

        private void HandleContextChanged(InputContext input, TargetContext target)
        {
            _inputContext = input;
            _targetContext = target;
            _conecaster = null;
            _inputContext.Descendant(ref _conecaster, optional: true);
            if (_conecaster) enabled = true;
            else enabled = false;
        }

        void LateUpdate()
        {
            ShowFov = ShowFov;
            ShowGrid = ShowGrid;

            var conecastResult = _conecaster.LastHit;

            var sphereScale = conecastResult.hitPointDistance * 2f * Vector3.one;

            _sphereRenderer.transform.SetPositionAndRotation(conecastResult.Cone.ray.ToPose().position, _selector.Rig.HeadPose.rotation);
            if(conecastResult.HasTarget) _sphereRenderer.transform.localScale = Vector3.Lerp(_sphereRenderer.transform.localScale, sphereScale, _distanceLerp * Time.deltaTime);

            for (var i = 0; i < _rings.Length; i++)
            {
                var ring = _rings[i];
                if (!ring.ShowRing)
                {
                    _material.SetVector(RingPositionId(i), Vector3.back);
                    continue;
                }

                ring.transform.Set(conecastResult.GazeEndPose);
                ring.RadiusDegrees = _conecaster.ConeRadiusDegrees;

                Vector3 cameraSpaceRingPosition = _sphereRenderer.transform.InverseTransformPoint(ring.transform.position);
                _material.SetVector(RingPositionId(i), cameraSpaceRingPosition);
                _material.SetFloat(RingBorderWidthId(i), ring.BorderWidth);
                _material.SetFloat(RingRadiusId(i), ring.RadiusDegrees);
                _material.SetColor(RingFillColorId(i), ring.FillColor);
                _material.SetColor(RingBorderColorId(i), ring.BorderColor);
            }
        }

        public Pose AverageRingPose()
        {
            Vector3 position = Vector3.zero;
            Vector3 direction = Vector3.zero;

            int shownRings = 0;
            foreach (var ring in _rings.Where(r => r.ShowRing))
            {
                shownRings++;
                position += ring.transform.position;
                direction += ring.transform.forward;
            }

            var rotation = direction.normalized == Vector3.zero ? Quaternion.identity : Quaternion.LookRotation(direction.normalized);
            return new Pose(position / Mathf.Max(shownRings, 1), rotation);
        }
    }
}
