using UnityEngine;

namespace Multiplayer.Runtime.View
{
    public class SyncedEntityTest : MonoBehaviour
    {
        private Camera _camera;
        private float _distanceFromCamera;
        private bool _isDragging;

        private void Awake()
        {
            _camera = Camera.main;
        }

        private void Update()
        {
            if (Input.GetMouseButtonDown(0))
            {
                var ray = _camera.ScreenPointToRay(Input.mousePosition);
                if (!Physics.Raycast(ray.origin, ray.direction, out var hit)) return;
                if (hit.transform != transform) return;
                _isDragging = true;
                _distanceFromCamera = (hit.point - _camera.transform.position).z;
            }
            else if (Input.GetMouseButton(0))
            {
                if (!_isDragging) return;
                var position = Input.mousePosition;
                position.z = _distanceFromCamera;
                transform.position = _camera.ScreenToWorldPoint(position);
            }
            else if(Input.GetMouseButtonUp(0))
            {
                _isDragging = false;
            }
        }
    }
}