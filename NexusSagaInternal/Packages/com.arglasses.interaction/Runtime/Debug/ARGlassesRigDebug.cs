using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [DefaultExecutionOrder(ExecutionOrder.ARGlassesRigDebug)]
    public class ARGlassesRigDebug : MonoBehaviour
    {
        [SerializeField] private bool _defaultActiveInLink = true;

        [SerializeField] private float _axisSize = 0.015f;
        [SerializeField] private float _lineWidth = 0.0015f;

        [SerializeField] private Color _neutralColor = new Color(0.1f, 0.1f, 0.1f);
        [SerializeField] private Color _indirectHoverColor = new Color(0.7f, 0.3f, 0.8f);
        [SerializeField] private Color _directHoverColor = new Color(0.2f, 0.9f, 0.9f);
        [SerializeField] private Color _selectColor = new Color(0.2f, 0.8f, 0.2f);

        [SerializeField] private Color _cvColor = Color.blue;
        [SerializeField] private Color _emgColor = Color.red;

        [SerializeField, ReadOnly] private ARGlassesRig _rig;
        [SerializeField, ReadOnly] private Camera _camera;
        [SerializeField, ReadOnly] private float _lastHoverDistance;

        [SerializeField, ReadOnly] private Selector _selector;
        [SerializeField, ReadOnly] private ContextPrioritizer _prioritizer;
        [SerializeField, ReadOnly] private bool _active;

        public event Action<bool> WhenActiveChanged = delegate { };

        private void Awake()
        {
            this.Scene(ref _selector);
            this.Scene(ref _prioritizer);
            this.Scene(ref _rig);
            _camera = Camera.main;
            _rig.WhenIsUserInHmdChanged += HandleUserInHmdChanged;
        }

        private void HandleUserInHmdChanged(bool inHeadset)
        {
            if (inHeadset && _rig.IsUserInLink) Active = _defaultActiveInLink;
        }

        public bool Active
        {
            get => _active;
            set
            {
                if (_active == value) return;
                _active = value;
                WhenActiveChanged(_active);
            }
        }

        private void LateUpdate()
        {
            if (_active) Draw(); // || !_rig.IsUserInHmd
        }

        private void Draw()
        {
            var targetContext = _selector.TargetContext;
            if (targetContext)
            {
                var collider = targetContext.Collider;
                DebugGizmos.Color = Color.green;
                Line.Draw(collider);
            }

            var inputContext = _selector.InputContext;
            if (inputContext is EyeGazeInputContext eyeGazeInteractionContext)
            {
                DrawConecast(_selector, eyeGazeInteractionContext.Conecaster, eyeGazeInteractionContext.ConecastResult);
            }

            if (inputContext is HandsInputContext handsInputContext)
            {
                DrawConecast(_selector, handsInputContext.Conecaster, handsInputContext.ConecastResult);
                DebugGizmos.Color = Color.blue;
                DebugGizmos.Draw(handsInputContext.PitchedRay);

                DebugGizmos.Color = Color.red;
                DebugGizmos.Draw(handsInputContext.AcceleratedRay);
            }

            if (inputContext is CursorInputContext cursorInputContext)
            {
                DrawConecast(_selector, cursorInputContext.Conecaster, cursorInputContext.ConecastResult);
            }

            Draw(_rig.LeftHandSnapshot);
            Draw(_rig.RightHandSnapshot);
            // DrawFocusManager(_rig.CardinalInteractor.Focus);
        }

        private void Draw(Snapshot.Hand snapshot)
        {
            var bones = snapshot.Bones;

            DebugGizmos.Color = Color.white * 0.1f;
            DebugGizmos.DrawLine(bones.Wrist.position, bones.IndexProximal.position);
            DebugGizmos.DrawLine(bones.Wrist.position, bones.ThumbProximal.position);

            DebugGizmos.Color = Color.white * 0.25f;
            DebugGizmos.DrawLine(bones.IndexProximal.position, bones.IndexDistal.position);
            DebugGizmos.DrawLine(bones.ThumbProximal.position, bones.ThumbDistal.position);

            DebugGizmos.Color = new Color(1, snapshot.IndexStrength, snapshot.MiddleStrength);
            DebugGizmos.DrawLine(bones.IndexDistal.position, bones.IndexTip.position);
            DebugGizmos.DrawLine(bones.ThumbDistal.position, bones.ThumbTip.position);

            DebugGizmos.Color = (snapshot.RayActive ? Color.green : Color.red) * 0.4f;
            DebugGizmos.Draw(snapshot.Ray);

            DebugGizmos.DrawAxis(snapshot.Pinch, 0.03f);

            if (_rig.IsUserInHmd)
            {
                DebugGizmos.Color = snapshot.IsTracked ? Color.green : Color.red;
                DebugGizmos.DrawLine(snapshot.Wrist.position, _rig.HeadPose.position + _rig.HeadPose.forward * 0.4f + _rig.HeadPose.up * -0.0075f);
            }
        }

        private void DrawBone(Pose a, Pose b)
        {
            DebugGizmos.DrawLine(a.position, b.position);
        }

        private void DrawConecast(Selector controller, Conecaster conecaster, ConecastResult conecastResult)
        {
            DebugGizmos.LineWidth = _lineWidth;

            var current = controller.Sequence.Latest;
            var begin = controller.Sequence.Begin;
            var selectorEvent = controller.LastSelectorEvent;



            var start = conecastResult.cone.ray.origin;
            var end = conecastResult.hitPoint;
            var rawPoint = conecastResult.gazePoint;
            var radiusDegrees = conecaster.ConeRadiusDegrees;

            DebugGizmos.DrawAxis(current.Head, _axisSize);

            if (controller.HasSelectionTarget)
            {
                var beginPointer = selectorEvent.Focus.TargetHit;
                var drag = selectorEvent.Selection.Drag;
                var latestPointer = beginPointer.AddWorldPosition(drag.DeltaWorld);
                DebugGizmos.DrawAxis(latestPointer, _axisSize);

                DebugGizmos.Color = _selectColor;
                DebugGizmos.LineWidth = _lineWidth;
                Line.Draw(current.Pinch.position, begin.Pinch.position);
                Line.Draw(latestPointer.position, beginPointer.position);

                DebugGizmos.DrawAxis(beginPointer, _axisSize);
            }
            else if (controller.HasHoverTarget)
            {
                DebugGizmos.Color = conecastResult.IsDirectHit ? _directHoverColor : _indirectHoverColor;
                DebugGizmos.LineWidth = _lineWidth * 0.95f;
                _lastHoverDistance = Vector3.Distance(start, rawPoint);
            }
            else
            {
                DebugGizmos.Color = _neutralColor;
                DebugGizmos.LineWidth = _lineWidth * 0.90f;
                var toRawPoint = rawPoint - start;

                for (int i = -2; i < 5; i++)
                {
                    var f = toRawPoint.normalized * 0.05f * i;
                    Line.AngularDegrees(start + (toRawPoint.normalized * _lastHoverDistance + f), radiusDegrees, _camera);
                }
            }

            if (!_rig.IsUserInHmd) Line.Draw(start, end);
            Line.AngularDegrees(rawPoint, radiusDegrees, _camera);
            DrawConecastResult(conecastResult, conecaster.ConeRadiusDegrees);

            // todo we shouldn't do this per interactor
            DebugGizmos.Color = _neutralColor;
            DebugGizmos.LineWidth = _lineWidth * 0.9f;
            var orderedHits = conecaster.OrderedHits;
            foreach (var hit in orderedHits) DrawConecastResult(hit, conecaster.ConeRadiusDegrees);
        }

        private void DrawConecastResult(ConecastResult conecastResult, float coneRadiusDegrees)
        {
            if (!conecastResult.IsDirectHit)
            {
                var hitPoint = conecastResult.hitPoint;
                var rawPoint = conecastResult.gazePoint;
                var normal = conecastResult.hitNormal;
                Line.AngularDegrees(rawPoint + normal * _lineWidth, coneRadiusDegrees, _camera);
                Line.Draw(hitPoint, rawPoint);
            }

            Line.Draw(conecastResult.target);
        }
    }
}
