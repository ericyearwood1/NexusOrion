using UnityEngine;

public class DemoSceneWaypoint : MonoBehaviour
{
    [SerializeField] private Transform _stopPoint;

    public Transform StopPoint => _stopPoint;
}
