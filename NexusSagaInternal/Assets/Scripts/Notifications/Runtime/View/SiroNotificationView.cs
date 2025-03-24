using Notifications.Runtime.Data;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Notifications.Runtime.View
{
    public class SiroNotificationView : MonoBehaviour
{
    protected const float MaxDisplayTime = 3f;
    protected const float DefaultFadeInTime = 0.4f;
    protected const float DefaultFadeOutTime = 0.4f;
    [SerializeField] private Sprite _successIcon;
    [SerializeField] private Sprite _errorIcon;
    [SerializeField] private Sprite _infoIcon;
    [SerializeField] private CanvasGroup _group;
    [SerializeField] private TMP_Text _text;
    [SerializeField] private Image _icon;
    [SerializeField] private ViewState _state;
    [SerializeField] private float _currentAnimationTime;
    private float _currentDisplayTime;
    private float _maxDisplayTime;

    public ViewState State => _state;

    public void Initialise(string text, NotificationType type)
    {
        _icon.sprite = GetIconForType(type);
        _text.text = text;
        _group.alpha = 0;
        _currentDisplayTime = 0;
        _currentAnimationTime = 0;
        _state = ViewState.AnimatingIn;
        _maxDisplayTime = MaxDisplayTime;
    }

    private Sprite GetIconForType(NotificationType notificationType)
    {
        switch (notificationType)
        {
            case NotificationType.Success: return _successIcon;
            case NotificationType.Error: return _errorIcon;
            case NotificationType.Info: return _infoIcon;
        }

        return null;
    }

    public void Hide()
    {
        _state = ViewState.AnimatingOut;
    }

    private void Update()
    {
        if (_state == ViewState.None) return;
        switch (_state)
        {
            case ViewState.AnimatingIn:
                AnimateIn();
                break;
            case ViewState.OnDisplay:
                _currentDisplayTime += Time.deltaTime;
                if (_currentDisplayTime >= _maxDisplayTime)
                {
                    Hide();
                }
                break;
            case ViewState.AnimatingOut:
                AnimateOut();
                break;
        }
    }

    protected virtual void UpdateDisplay(float progress)
    {
        _group.alpha = progress;
    }

    private void AnimateOut()
    {
        _currentAnimationTime += Time.deltaTime;
        var progress = 1 - Mathf.Max(0, _currentAnimationTime / DefaultFadeOutTime);
        UpdateDisplay(progress);
        if (progress <= 0)
        {
            AnimateOutComplete();
        }
    }

    private void AnimateIn()
    {
        _currentAnimationTime += Time.deltaTime;
        var progress = Mathf.Max(0, _currentAnimationTime / DefaultFadeInTime);
        UpdateDisplay(progress);
        if (progress >= 1)
        {
            AnimateInComplete();
        }
    }

    private void AnimateInComplete()
    {
        _currentAnimationTime = 0;
        _state = ViewState.OnDisplay;
    }

    private void AnimateOutComplete()
    {
        _currentAnimationTime = 0;
        _state = ViewState.Hidden;
    }

    public void Reset()
    {
        _state = ViewState.None;
        _group.alpha = 0;
        _maxDisplayTime = MaxDisplayTime;
    }

    public void SetDisplayTime(float time)
    {
        _maxDisplayTime = time;
    }
}
}
