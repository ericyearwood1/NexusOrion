using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UnityOrionFloatFixer : MonoBehaviour
{
    [SerializeField] private Transform peggedPosition;
    [SerializeField] private Queue<Vector3> positions;
    [SerializeField] private Queue<Quaternion> rotations;
    private int frameLag = 1;

    private void Start()
    {
        positions = new Queue<Vector3>();
        rotations = new Queue<Quaternion>();
    }
    private void Update()
    {
        positions.Enqueue(peggedPosition.position);
        rotations.Enqueue(peggedPosition.rotation);
        while(positions.Count > frameLag)
        {
            transform.position = Vector3.Lerp(peggedPosition.position, positions.Dequeue(), 0.9f);
            transform.rotation = Quaternion.Lerp(peggedPosition.rotation, rotations.Dequeue(), 0.9f);
        }
    }
}
