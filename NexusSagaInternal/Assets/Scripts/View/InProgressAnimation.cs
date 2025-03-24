using UnityEngine;

public class InProgressAnimation : MonoBehaviour
{
    [SerializeField] private Transform _indicator;
    [SerializeField] private float _speed;
    private void Start()
    {
        if (_indicator != null) return;
        Debug.LogError("In progress animation image is null. Destroying GO");
        Destroy(gameObject);
    }

    private void Update()
    {
        var eulerAngles = _indicator.localEulerAngles;
        eulerAngles.z += _speed * Time.deltaTime;
        _indicator.localEulerAngles = eulerAngles;
    }
}
