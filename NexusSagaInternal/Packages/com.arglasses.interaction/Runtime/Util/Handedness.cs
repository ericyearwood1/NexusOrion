namespace ARGlasses.Interaction
{
    public static class ExtensionsHandedness
    {
        public static bool IsLeft(this Handedness h) => h == Handedness.Left;
        public static bool IsRight(this Handedness h) => h == Handedness.Right;
        public static Handedness Other(this Handedness h) => h.IsRight() ? Handedness.Left : Handedness.Right;
    }

    public enum Handedness
    {
        Unsupported = OVRPlugin.Handedness.Unsupported,
        Left = OVRPlugin.Handedness.LeftHanded,
        Right = OVRPlugin.Handedness.RightHanded,
    }
}
