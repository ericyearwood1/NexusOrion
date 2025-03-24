using System;
using System.Collections;
using ARGlasses.Interaction;
using OSIG.Tools.Units;
using UnityEngine;

public class ARGlassesPersonalSpace : MonoBehaviour
{
    public float _distanceFromEye = 1f;
    public float DistanceFromEye
    {
        get => _distanceFromEye;
        set
        {
            if (_distanceFromEye == value) return;
            _distanceFromEye = value;
            if (_ocUnitSettings) _ocUnitSettings.ViewDistance = _distanceFromEye;
            ResetPosition();
        }
    }

    public float _heightFromEyeInGazeMode = -0.1f;
    public float _heightFromEyeInHandsMode = -0.2f;

    [SerializeField, ReadOnly] private Selector _selector;
    [SerializeField, ReadOnly] private CanvasGroup _group;
    [SerializeField, ReadOnly] private float _alphaTarget;
    [SerializeField, ReadOnly] private OCUnitsSettings _ocUnitSettings;

    public event Action WhenPositionChanged = delegate { };

    private void Awake()
    {
        this.Scene(ref _selector);
        this.Sibling(ref _ocUnitSettings, optional: true);
        this.Ensure(ref _group);
        _group.alpha = 0;
    }

    private void OnDisable() => _selector.WhenInputCategoryChanged -= CategoryChanged;

    private void CategoryChanged(InputCategory category) => ResetPosition();

    public IEnumerator Start()
    {
        yield return new WaitForSeconds(1f);
        ResetPosition();
    }

    void OnEnable()
    {
        _selector.WhenInputCategoryChanged += CategoryChanged;
        ResetPosition();
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.F11)) ResetPosition();
        if (_group.alpha.IsNotApproximately(_alphaTarget)) _group.alpha = Mathf.Lerp(_group.alpha, _alphaTarget, Time.deltaTime * 4f);
    }

    public void ResetPosition()
    {
        _group.alpha = 0;
        _alphaTarget = 1;
        var camera = Camera.main.transform;
        var heightFromEye = _selector.Category.IsHands() ? _heightFromEyeInHandsMode : _heightFromEyeInGazeMode;

        var position = camera.position + camera.forward.WithY(0) * _distanceFromEye + Vector3.up * heightFromEye;
        var rotation = Quaternion.LookRotation((position - camera.position).WithY(0));
        transform.SetPositionAndRotation(position, rotation);

        WhenPositionChanged();
    }
}
