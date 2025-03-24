using UnityEngine;

public class RecordAnimateRing : MonoBehaviour
{
    public Vector3 rotation = new Vector3(0, 0, 1);
    public float rotationSpeed = 50f;

    private void Update()
    {
        var rotate = rotation * (-rotationSpeed * Time.deltaTime);
        transform.Rotate(rotate.x, rotate.y, rotate.z);
    }
}
