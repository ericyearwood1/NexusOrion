using System;
using ARGlasses.Interaction;
using ARGlasses.Interaction.Motion;
using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using ProtoKit.GraphicBase;
using ProtoKit.UI;
using UnityEngine;

namespace ARGlasses.Components
{
    public class ArgCursorEffect : MonoBehaviour, ICursorEffect, IOCLayoutListener
    {
        [SerializeField] private BackgroundProvider _backgroundProvider;
        [SerializeField] private RoundedRect _effectPanel;
        [SerializeField] private OCLayoutSize _layout;
        [SerializeField] private OCInsetAttachData _attachData;

        private Pose _pose;
        private bool _overwritePose;

        public ArgIcon _icon;
        public PKUIPanel _backgroundPanel;

        private void Awake()
        {
            _nudgeMotionLocal = new Motion<Vector3>(Vector3.zero, _nudgeParams);
            this.Descendant(ref _backgroundProvider, optional: true, allowSibling: true, includeInactive: true);
            if (!_backgroundProvider)
            {
                enabled = false;
                return;
            }

            _backgroundPanel = _backgroundProvider.GetComponent<PKUIPanel>();
            _icon = _backgroundProvider.GetComponentInChildren<ArgIcon>();

            _backgroundPanel.CreateChild(ref _effectPanel);
            _effectPanel.SetColorA(Color.clear);
            _effectPanel.transform.SetSiblingIndex(0);
            _effectPanel.SetRadius(CornerRadius);

            _effectPanel.Ensure(ref _layout);
            _effectPanel.Ensure(ref _attachData);
            _attachData.Set2DAlignment(OCLayoutPadding.Alignment.CenterFill);
        }

        [SerializeField] private float _nudgeClamp = 0.0025f;
        float ICursorEffect.NudgeClamp => _nudgeClamp; //smaller numbers for less nudging
        private readonly int _defaultCornerRadiusPixels = 24;
        private OCValue CornerRadius => _backgroundProvider != null ? _backgroundProvider.BackgroundRenderer.Radius : _defaultCornerRadiusPixels.AsPixels();
        public float GetRadiusPixels() => _effectPanel ? CornerRadius.GetPixels(_effectPanel.UnitsContext) : _defaultCornerRadiusPixels;

        [SerializeField, ReadOnly] private Motion<Vector3> _nudgeMotionLocal;
        [SerializeField] private MotionParams _nudgeParams = AMotionParams.Default;

        private void Update()
        {
            var nudgePosition = _nudgeMotionLocal.Step(Time.deltaTime);
            if (nudgePosition.IsApproximately(_backgroundPanel.transform.localPosition)) return;
            _backgroundPanel.transform.localPosition = nudgePosition;
        }

        public void SetPose(Pose pos)
        {
            _pose = pos;
            _overwritePose = true;
        }

        public void ResetPose() => _overwritePose = false;
        public void SetColor(Color color)
        {
            if(_effectPanel) _effectPanel.SetColorA(color);
        }

        public void SetLayoutSizePixels(Vector2 size)
        {
            if(_layout) _layout.SetSize2D(size);
        }

        public void OnApplyLayout(IOCLayoutComponent layoutComponent)
        {
            if (_overwritePose && _effectPanel) _effectPanel.transform.SetPositionAndRotation(_pose.position, _pose.rotation);
        }

        public void SetIconPosition(Vector3 position)
        {
            if (_icon) _icon.transform.position = position;
        }
        public void SetIconScale(Vector3 scale)
        {
            if (_icon) _icon.transform.localScale = scale;
        }

        public void SetBackgroundOffset(Vector3 localOffset)
        {
            _nudgeMotionLocal.SetGoal(localOffset);
        }

        public void SetBackgroundScale(Vector3 scale)
        {
            if(_backgroundPanel) _backgroundPanel.transform.localScale = scale;
        }

        public void SetBackgroundEnabled(bool state)
        {
            if(_backgroundPanel) _backgroundPanel.enabled = state;
        }
    }
}
