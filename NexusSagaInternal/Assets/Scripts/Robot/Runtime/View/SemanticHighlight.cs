using Robot.Runtime.Data;
using UnityEngine;

public class SemanticHighlight : MonoBehaviour
{
    [SerializeField] private SpriteRenderer[] _fadeRenderers;
    [SerializeField] private Transform[] _billboards;
    [SerializeField] private Transform _worldUpContainer;

    protected const float DefaultFadeInTime = 0.4f;
    protected const float DefaultFadeOutTime = 0.4f;

    [SerializeField] private bool _isBillboard = true;
    [SerializeField] private float _maxAlpha = 1.0f;

    private Transform _cameraTransform;
    protected float _currentFadeTime;
    private HighlightState _state = HighlightState.OnDisplay;
    private float _fadeInTime;
    protected float _fadeOutTime;

    public HighlightState State => _state;

    protected virtual void StartShow(Transform cameraTransform, float fadeInTime = DefaultFadeInTime)
    {
        _fadeInTime = fadeInTime;
        _cameraTransform = cameraTransform;
        _currentFadeTime = 0;
        _state = HighlightState.AnimatingIn;
        gameObject.SetActive(true);
    }

    public virtual void Show(Transform cameraTransform)
    {
        StartShow(cameraTransform);
    }

    public void Dispose()
    {
        _cameraTransform = null;
    }

    private void Update()
    {
        if (_state == HighlightState.None) return;
        switch (_state)
        {
            case HighlightState.AnimatingIn:
                AnimateIn();
                break;
            case HighlightState.OnDisplay:
                break;
            case HighlightState.AnimatingOut:
                AnimateOut();
                break;
        }

        _worldUpContainer.up = Vector3.up;
        Billboard();
    }

    protected virtual void UpdateDisplay(float progress)
    {
        foreach (var display in _fadeRenderers)
        {
            display.color = new Color(1, 1, 1, progress * _maxAlpha);
        }
    }

    private void AnimateOut()
    {
        _currentFadeTime += Time.deltaTime;
        var progress = 1 - Mathf.Max(0, _currentFadeTime / _fadeOutTime);
        UpdateDisplay(progress);
        if (progress <= 0)
        {
            AnimateOutComplete();
        }
    }

    private void AnimateIn()
    {
        _currentFadeTime += Time.deltaTime;
        var progress = Mathf.Max(0, _currentFadeTime / _fadeInTime);
        UpdateDisplay(progress);
        if (progress >= 1)
        {
            AnimateInComplete();
        }
    }

    private void AnimateInComplete()
    {
        _currentFadeTime = 0;
        _state = HighlightState.OnDisplay;
    }

    private void AnimateOutComplete()
    {
        _currentFadeTime = 0;
        _state = HighlightState.Hidden;
    }

    private void Billboard()
    {
        if (!_isBillboard || _cameraTransform == null) return;
        foreach (var billboard in _billboards)
        {
            var lookRotation = Quaternion.LookRotation(billboard.position - _cameraTransform.position);
            lookRotation = Quaternion.Euler(new Vector3(0f, lookRotation.eulerAngles.y, 0f));
            billboard.rotation = lookRotation;
        }
        
    }

    public virtual void Reset()
    {
        UpdateDisplay(0);
        gameObject.SetActive(false);
        _state = HighlightState.None;
    }

    public virtual void Hide(float fadeOutTime = DefaultFadeOutTime)
    {
        if (_state != HighlightState.AnimatingIn && _state != HighlightState.OnDisplay) return;
        _fadeOutTime = fadeOutTime;
        _state = HighlightState.AnimatingOut;
    }

    public virtual void OnInstructionComplete()
    {
        Hide();
    }

    public void OnActionComplete()
    {
        Hide();
    }
}