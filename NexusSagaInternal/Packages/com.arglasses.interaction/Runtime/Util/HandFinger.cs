namespace ARGlasses.Interaction
{
    public enum HandFinger
    {
        Thumb = OVRPlugin.HandFinger.Thumb,
        Index = OVRPlugin.HandFinger.Index,
        Middle = OVRPlugin.HandFinger.Middle,
        Ring = OVRPlugin.HandFinger.Ring,
        Pinky = OVRPlugin.HandFinger.Pinky,
        Max = OVRPlugin.HandFinger.Max,
    }

    public static class ExtensionsHandFinger
    {
        public static bool IsThumb(this HandFinger digit) => digit == HandFinger.Thumb;
        public static bool IsIndex(this HandFinger digit) => digit == HandFinger.Index;
        public static bool IsMiddle(this HandFinger digit) => digit == HandFinger.Middle;
        public static bool IsAny(this HandFinger digit) => digit.IsThumb() || digit.IsIndex() || digit.IsMiddle();
    }
}
