using ARGlasses.Interaction.Motion;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class BasicCursorEffect : MonoBehaviour, ICursorEffect
    {
        [SerializeField] private Transform _displacement;
        [SerializeField] float _cornerRadiusPixels = 24f;
        [SerializeField] private float _springBackLerp = 4f;
        [SerializeField] private float _nudgeClamp = 0.0025f;
        float ICursorEffect.NudgeClamp => _nudgeClamp; //smaller numbers for less nudging

        [SerializeField, ReadOnly] private Motion<Vector3> _nudgeMotionLocal;
        [SerializeField] private MotionParams _nudgeParams = AMotionParams.Default;

        public float GetRadiusPixels() => _cornerRadiusPixels;

        public void SetBackgroundOffset(Vector3 localOffset) => _nudgeMotionLocal.SetGoal(localOffset);

        private void Awake()
        {
            _nudgeMotionLocal = new Motion<Vector3>(_displacement ? _displacement.transform.localPosition : Vector3.zero, _nudgeParams);
        }

        private void Update()
        {
            if(_displacement) _displacement.transform.localPosition = _nudgeMotionLocal.Step(Time.deltaTime);
        }
    }
}
