using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class HandTrackingLossSimulation : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ARGlassesRig _rig;
        [SerializeField, ReadOnly] private OVRCameraRig _ovrCameraRig;

        [SerializeField, ReadOnly] private List<TrackingCone> _cones;
        [SerializeField] private bool _drawDebug;

        private void Start()
        {
            this.Scene(ref _rig);
            this.Scene(ref _ovrCameraRig);

            var centerEyeAnchor = _ovrCameraRig.centerEyeAnchor;

            var downConeLeft = centerEyeAnchor.CreateChild<TrackingCone>(name: "DownConeLeft");
            downConeLeft.DebugColor = Color.cyan;
            downConeLeft.DrawDebug = _drawDebug;
            downConeLeft.SetAngles(70, 90);
            downConeLeft.transform.rotation = Quaternion.Euler(90, 0, 0);
            downConeLeft.transform.localPosition = new Vector3(-0.07f, 0.004f, 0.016f);

            var downConeRight = centerEyeAnchor.CreateChild<TrackingCone>(name: "DownConeRight");
            downConeRight.DebugColor = Color.magenta;
            downConeRight.DrawDebug = _drawDebug;
            downConeRight.SetAngles(70, 90);
            downConeRight.transform.rotation = Quaternion.Euler(90, 0, 0);
            downConeRight.transform.localPosition = new Vector3(0.07f, 0.004f, 0.016f);

            var frontConeRight = centerEyeAnchor.CreateChild<TrackingCone>(name: "FrontConeRight");
            frontConeRight.DebugColor = Color.yellow;
            frontConeRight.DrawDebug = _drawDebug;
            frontConeRight.SetAngles(110, 110);
            frontConeRight.transform.rotation = Quaternion.Euler(6, 13, 0);
            frontConeRight.transform.localPosition = new Vector3(0.06f, 0.014f, 0.004f);

            // no frontConeLeft!

            _cones.Add(downConeLeft);
            _cones.Add(downConeRight);
            _cones.Add(frontConeRight);
        }

        public bool Contained(Vector3 position)
        {
            var wrist = position;
            var isContained = false;
            for (int i = 0; i < _cones.Count; i++) if (_cones[i].Contains(wrist)) isContained = true;
            return isContained;
        }
    }
}
