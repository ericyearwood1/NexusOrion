
namespace ARGlasses.Interaction
{
#if false
    public class ITKHandRay : MonoBehaviour
    {
        (Filter<Quaternion> Rotation,
            Filter<Vector3> PinchPosition,
            Filter<Vector3> WristPosition,
            Filter<Vector3> OffsetWristPosition,
            Filter<Quaternion> PointerPoseOrientationOffset) Filters;
        SmoothLatch<float> gazeLatch;

        Vector3 initialPinchPosition;
        Quaternion initialPointerPoseOrientation;
        Quaternion pointerPoseOrientationOffset;
        bool intendToMove = false;
        bool wasPinching = false;
        Vector3 pinchOffset;
        Vector3 stablePinchPosition;

        // stored so DrawDebugVisuals can be kept independent of the logic
        float gazeAngle, gazeContrib, latchedContrib;
        Vector3 gazeHipShoulderPoint, stabilizedHipShoulderPoint, wristPoint;
        Quaternion wristOrientation, bodyOrientation, telescopeDir, rawPointerPoseOrientation;
        bool wasHandTracked = false;

        bool FilterHandChirality(HandSkeleton c) => c.Chirality == Chirality;

        protected override void Start()
        {
            Reset();
        }

        void ResetFilters()
        {
            if (TargetingSettings)
            {
                Filters = (TargetingSettings.RotationFilter.CreateFilter<Quaternion>(),
                    FilterParameters.CreateFilter<Vector3>(null),/*TargetingSettings.PositionFilter.CreateFilter<Vector3>(),*/ //hack to prevent popping during teleport
                    TargetingSettings.PositionFilter.CreateFilter<Vector3>(),
                    TargetingSettings.PositionFilter.CreateFilter<Vector3>(), // important to use the same params as the wrist pos
                    TargetingSettings.PointerPoseOrientationOffsetFilter.CreateFilter<Quaternion>());
            }
        }

        public void Reset()
        {
            ResetFilters();
            pinchOffset = TargetingSettings ? TargetingSettings.DefaultPinchOffset : Vector3.zero;
            if (!HandSkeleton.IsLeft)
            {
                pinchOffset *= -1;
            }
        }

        // roughly corresponds to PointerPoseModel::Update in
        //  Software/OculusSDK/Mobile/VrRuntime/Common/Input/PointerPoseModel.h
        //  For any logic here that doesn't have a corresponding c++ logic in shell, it has been marked (ITK ONLY)
        void Update()
        {
            //When we lose tracking, reset the filters/stabilization and early return
            if (!HandSkeleton.IsTracked)
            {
                if (wasHandTracked)
                {
                    ResetFilters();
                    pointerPoseOrientationOffset = Vector4.zero.ToQuat();
                    intendToMove = false;
                    wasHandTracked = false;
                    wasPinching = false;
                }
                return;
            }
            wasHandTracked = true;
            float dt = Time.deltaTime;

            Digit primaryDigit = (ManipulatorStrengthController != null ? ManipulatorStrengthController.GetPrimaryButtonDigit() : Digit.Index);
            bool isPinching = HandSkeleton.IsPinching(primaryDigit);
            Vector3 currentPinchOffset = HandSkeleton.Wrist.InverseTransformPoint(Vector3.Lerp(HandSkeleton.Thumb.Tip.position, HandSkeleton.Index.Tip.position, 0.5f));

            // NormalizePinchOffset (ITK ONLY)
            if (TargetingSettings.NormalizePinchOffset)
            {
                currentPinchOffset *= 1.5f * HandSkeleton.Thumb.Length / currentPinchOffset.magnitude; // keep it the same distance from the wrist
            }

            // Step 0 ===========================
            // Update the stable pinch point

            if (wasPinching && !isPinching)
            {
                // Only update the pinch offset upon release, so the cursor doesn't 'drift' while
                // pinching

                if (TargetingSettings.AutoCalculatePinchOffset)
                {
                    pinchOffset = Vector3.Lerp(
                        pinchOffset, currentPinchOffset, TargetingSettings.PinchOffsetBlend);
                }

                // We reset PointerPoseOrientationOffsetFilter upon release and update it with the
                // latest PointerPoseOrientationOffset

                // NOTE: should probably have something more like Filter.Reset() like we have in OVR_Filter.h
                Filters.PointerPoseOrientationOffset = TargetingSettings.PointerPoseOrientationOffsetFilter.CreateFilter<Quaternion>();
                pointerPoseOrientationOffset =
                    Filters.PointerPoseOrientationOffset(pointerPoseOrientationOffset, dt);
            }

            // filter the tracking space position so that slight wrist rotation doesn't cause 'drift'
            stablePinchPosition = Filters.PinchPosition(HandSkeleton.Wrist.TransformPoint(pinchOffset), dt);

            // update the hand skeleton (ITK ONLY)
            HandSkeleton.Skeleton.BindSkeletalFrame(HandSkeleton.GetSkeletalFrame(HandFrame.StablePinch), new Pose(stablePinchPosition, RayManipulator.transform.rotation));

            // Step 1 ===========================
            // Gaze determines a point between hip and shoulder
            var head = HandSkeleton.Skeleton.Head.CenterEye;
            gazeAngle = Mathf.DeltaAngle(head.eulerAngles.x, 0f);
            gazeContrib = 1f - Mathf.InverseLerp(TargetingSettings.GazeRange.x,
                TargetingSettings.GazeRange.y,
                gazeAngle);
            gazeHipShoulderPoint = GetHipShoulderPoint(gazeContrib);

            // Step 2 ===========================
            // Velocity latches the hipShoulderPoint
            if (gazeLatch == null) gazeLatch = new SmoothLatch<float>();

            latchedContrib = gazeLatch.Update(gazeContrib, !HandSkeleton.Classifier.IsWristMoving || HandSkeleton.IsPinching(primaryDigit), TargetingSettings.GazeTransitionSpeed * Time.deltaTime);
            var chiral = HandSkeleton.Skeleton.Chiral(Chirality);
            var bodyOffset = HandSkeleton.IsLeft ? TargetingSettings.BodyOffset : -TargetingSettings.BodyOffset;
            stabilizedHipShoulderPoint = GetHipShoulderPoint(latchedContrib) + chiral.Hip.TransformVector(new Vector3(bodyOffset.x, bodyOffset.y, 0f));

            // Step 3 ===========================
            // wrist

            var up = HandSkeleton.Wrist.TransformDirection(0, 1, 0);

            // Filtering the wrist position gives better stability of the body ray
            var wrist = Filters.WristPosition(HandSkeleton.Wrist.position, dt);
            // Filtering the offset rotation combats wrist rotation wiggles
            // It's also important that we filter the wristPoint the same as the StablePinchPoint
            // or else the difference in lag will add strange rotations to the ray.
            var wristOffset = HandSkeleton.IsLeft ? TargetingSettings.WristOffset : -TargetingSettings.WristOffset;
            wristOffset = new Vector3(0f, -wristOffset.y, -wristOffset.x); // swizzled because hand has strange orientations
            wristPoint = Filters.OffsetWristPosition(HandSkeleton.Wrist.TransformPoint(wristOffset), dt);
            var wristDirection = stablePinchPosition - wristPoint;
            wristOrientation = Quaternion.LookRotation(wristDirection, up);

            // Step 4 ===========================
            // hip/shoulder direciton
            var bodyDirectionTarget = Vector3.Lerp(wrist, stablePinchPosition, TargetingSettings.PinchContribution);
            var bodyDirection = bodyDirectionTarget - stabilizedHipShoulderPoint;
            bodyOrientation = Quaternion.LookRotation(bodyDirection, up);

            // Step 4.1 ==============
            // telescope direction (ITK ONLY)
            telescopeDir = Quaternion.LookRotation(stablePinchPosition - head.position, up);

            // Step 4.2 =========================
            // mix the three directions based on their respective contributions.
            rawPointerPoseOrientation = Quaternion.Slerp(
                Quaternion.Slerp(bodyOrientation, wristOrientation, TargetingSettings.WristContribution),
                telescopeDir,
                TargetingSettings.TelescopeContribution);

            // NOTE: Depending on the magnitude of the offset, this may be adding undesirable rotational inaccuracy.
            //  Keeping for now to get parity with shell
            var pointerPoseOrientationFinal = MathEx.FastAdd(pointerPoseOrientationOffset, rawPointerPoseOrientation);

            // we update the initial pinch position and initial pointer pose orientation if last frame
            // is not pinching
            if (!wasPinching)
            {
                initialPinchPosition = stablePinchPosition;
                initialPointerPoseOrientation = pointerPoseOrientationFinal;
                intendToMove = false;
            }

            // we heavily stabilize the pointer pose orientation if there is no intention to move and
            // user is pinching

            if (!intendToMove && isPinching)
            {
                float pinchPosDelta = (stablePinchPosition - initialPinchPosition).magnitude;
                // create a dead zone around the initial pinch
                if (pinchPosDelta > TargetingSettings.IntendToMoveDeltaThresh)
                {
                    // if the distance between the pinch position and initial pinch position goes beyond
                    // the threshold, we assume that the hand is intending to move
                    intendToMove = true;
                }
                else if (Vector3.Angle(initialPointerPoseOrientation * Vector3.up, pointerPoseOrientationFinal * Vector3.up) >
                         TargetingSettings.IntendToMoveRotationThresh)
                {
                    // if the rotation of the wrist goes beyond the threshold, we stop stabilizing (ITK ONLY)
                    intendToMove = true;
                }
                else
                {
                    // if the hand is not intended to move, we blend the raw pointer pose orientation
                    // with the initial pointer pose orientation, the blending factor is computed as a
                    // sigmoid function
                    float rawPointerPoseContribution = 2.0f /
                                                       (1.0f + Mathf.Exp((TargetingSettings.IntendToMoveDeltaThresh - pinchPosDelta) *
                                                                         TargetingSettings.DeadZoneSteepness));

                    pointerPoseOrientationFinal = Quaternion.Slerp(
                        initialPointerPoseOrientation,
                        rawPointerPoseOrientation,
                        rawPointerPoseContribution);
                    // We also update the pointer pose orientation offset, i.e. the difference between
                    // the final pointer pose orientation and the raw pointer pose orientation

                    // NOTE: Depending on the magnitude of the offset, this may be adding undesirable rotational inaccuracy.
                    //  Keeping for now to get parity with shell
                    pointerPoseOrientationOffset =
                        MathEx.FastSubtract(pointerPoseOrientationFinal, rawPointerPoseOrientation);
                }
            }

            // we smoothly transition the pointer pose orientation offset to 0 when not pinching to
            // avoid sudden orientation change
            if (!isPinching)
            {
                pointerPoseOrientationOffset = Filters.PointerPoseOrientationOffset(Vector4.zero.ToQuat(), dt);
            }

            // Step 5 ============================
            // filter and profit
            pointerPoseOrientationFinal = Filters.Rotation(pointerPoseOrientationFinal, Time.deltaTime);

            RayManipulator.transform.SetPositionAndRotation(stablePinchPosition, pointerPoseOrientationFinal);
        }

        protected Vector3 GetHipShoulderPoint(float gazeContrib)
        {
            return Vector3.Lerp(Hip.position, Shoulder.position, Mathf.Lerp(TargetingSettings.BodyRange.x, TargetingSettings.BodyRange.y, gazeContrib));
        }

    }

#endif
}
