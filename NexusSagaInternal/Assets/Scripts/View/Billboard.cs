using UnityEngine;

public class Billboard : MonoBehaviour
{
    private Transform _cameraTransform;
    [SerializeField] private bool _isOnlyY = true;

    private void Awake()
    {
        _cameraTransform = Camera.main.transform;
    }
    
    private void Update()
    {
        var lookRotation = Quaternion.LookRotation(transform.position - _cameraTransform.position);
        if (_isOnlyY)
        {
            lookRotation = Quaternion.Euler(new Vector3(0f, lookRotation.eulerAngles.y, 0f));
        }
        transform.rotation = lookRotation;
    }
}
