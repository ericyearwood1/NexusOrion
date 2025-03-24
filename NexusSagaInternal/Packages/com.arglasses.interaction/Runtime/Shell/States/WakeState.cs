namespace ARGlasses.Interaction
{
    public enum WakeState
    {
        Sleep,
        Awake
    }

    public static class ExtensionWakeState
    {
        public static bool IsAwake(this WakeState s) => s == WakeState.Awake;
        public static bool IsSleep(this WakeState s) => s == WakeState.Sleep;
    }
}
