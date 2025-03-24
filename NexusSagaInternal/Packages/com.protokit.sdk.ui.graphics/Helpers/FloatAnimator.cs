using System.Collections;
using UnityEngine;
using UnityEngine.Events;

[AddComponentMenu("OSIG Tools/Float Animator")]
public class FloatAnimator : MonoBehaviour {
    [System.Serializable]
    public class FloatUnityEvent : UnityEvent<float> { }

    public float BeginValue = 0f;
    public float FinalValue = 0f;
    [Space]
    public BezierAnimationCurve InCurve;
    public float InDuration = 0.2f;
    [Space]
    public BezierAnimationCurve OutCurve;
    public float OutDuration = 0.2f;
    [Space]

    public UnityEvent OnBegin = null;
    public FloatUnityEvent OnUpdate = null;
    public UnityEvent OnComplete = null;

    private AnimationCurve curve;
    private float duration = 0f;

    private float beginValue = 0f;
    private float finalValue = 0f;
    private float value = 0f;

    public void AnimateForward() {
        beginValue = value;
        finalValue = FinalValue;
        duration = InDuration;
        curve = (AnimationCurve)InCurve;
        Animate();
    }

    public void AnimateBack() {
        beginValue = value;
        finalValue = BeginValue;
        duration = OutDuration;
        curve = (AnimationCurve)OutCurve;
        Animate();
    }

    public void ResetAnimation() {
        value = BeginValue;
        OnUpdate?.Invoke(value);
    }

    private void Animate() {
        OnBegin?.Invoke();
        StopCoroutine(OnAnimate());
        StartCoroutine(OnAnimate());
    }

    IEnumerator OnAnimate() {
        float time = 0f;

        while (time < duration) {
            float t = curve.Evaluate(time / duration);
            value = Mathf.Lerp(beginValue, finalValue, t);
            OnUpdate?.Invoke(value);

            time += Time.deltaTime;
            yield return new WaitForEndOfFrame();
        }
        // make sure we end on the final value
        value = finalValue;
        OnUpdate?.Invoke(value);
        OnComplete?.Invoke();
    }
}
