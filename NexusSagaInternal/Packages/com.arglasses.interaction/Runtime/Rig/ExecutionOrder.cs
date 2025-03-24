namespace ARGlasses.Interaction
{
    /// We use [DefaultExecutionOrder] to make sure Awake, OnEnable and Update
    /// are called in the correct order relative to "upstream" collaborators
    public class ExecutionOrder
    {
        public const int OVRHands = -90;
        public const int FromOVRHandTracking = OVRHands + 2;

        // CTRLClient -85

        public const int OVREyeGaze = -80;
        public const int FromOVREyeTracking = OVREyeGaze + 2;

        public const int ARGlassesWristband = -66;
        public const int ARGlassesWristbandConsumers = ARGlassesWristband + 1;
        public const int ARGlassesFingerRig = ARGlassesWristband + 2; // after Hands and ARGlassesWristband

        public const int OVRCameraRig = -50;
        public const int ARGlassesHandRig = OVRCameraRig + 2;
        public const int ARGlassesRig = ARGlassesHandRig + 2;

        public const int MouseKeyboardEmulator = ARGlassesRig + 2;
        public const int ARGlassesRigDebug = MouseKeyboardEmulator + 2;


        // Evaluate InputProcessing after any custom components have run their Updates for the frame
        public const int HoverSelectionEvents = 40;

        public const int SessionWriting = 1000;
        public const int SessionReading = ARGlassesRig - 1;
    }
}
