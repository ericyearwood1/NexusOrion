using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Follow3DView : MonoBehaviour
{
    public Camera leftEyeCam, rightEyeCam;
    public Transform leftEye, rightEye;
    public Transform target;
    public OVROverlay overlay;

    // Update is called once per frame
    void Update()
    {
        leftEyeCam.transform.position = leftEye.position;
        rightEyeCam.transform.position = rightEye.position;
        overlay.transform.position = target.position;

        Vector3 center = (leftEye.transform.position + rightEye.transform.position) / 2f;
        overlay.transform.LookAt(overlay.transform.position * 2f - center, Vector3.up);
        leftEyeCam.transform.LookAt(overlay.transform.position, overlay.transform.up);
        rightEyeCam.transform.LookAt(overlay.transform.position, overlay.transform.up);

        float dist = Vector3.Distance(overlay.transform.position, leftEyeCam.transform.position);

        leftEyeCam.fieldOfView = Mathf.Atan(overlay.transform.localScale.x / 2f / dist) * 2f * Mathf.Rad2Deg;
        rightEyeCam.fieldOfView = Mathf.Atan(overlay.transform.localScale.x / 2f / Vector3.Distance(overlay.transform.position, rightEyeCam.transform.position)) * 2f * Mathf.Rad2Deg;
        leftEyeCam.nearClipPlane = Mathf.Max(0.01f, dist - overlay.transform.localScale.x / 2f);
        leftEyeCam.farClipPlane = dist + overlay.transform.localScale.x / 2f;
        rightEyeCam.nearClipPlane = Mathf.Max(0.01f, dist - overlay.transform.localScale.x / 2f);
        rightEyeCam.farClipPlane = dist + overlay.transform.localScale.x / 2f;

    }
}
