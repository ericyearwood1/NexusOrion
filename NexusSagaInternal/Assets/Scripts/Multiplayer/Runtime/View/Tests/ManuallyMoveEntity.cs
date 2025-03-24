using UnityEngine;

public class ManuallyMoveEntity : MonoBehaviour
{
    private Camera _mainCamera;
    private bool _isDragging;
    private float _distance;

    private void Awake()
    {
        _mainCamera = Camera.main;
    }

    private void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            var ray = _mainCamera.ScreenPointToRay(Input.mousePosition);
            if (!Physics.Raycast(ray.origin, ray.direction, out var hit)) return;
            if (hit.transform != transform) return;
            _isDragging = true;
            _distance = hit.transform.position.z;
        }
        else if (Input.GetMouseButtonUp(0))
        {
            _isDragging = false;
        }
        else if (_isDragging && Input.GetMouseButton(0))
        {
            var targetPosition = Input.mousePosition;
            targetPosition.z = _distance;
            transform.position = _mainCamera.ScreenToWorldPoint(targetPosition);
        }
    }
}
