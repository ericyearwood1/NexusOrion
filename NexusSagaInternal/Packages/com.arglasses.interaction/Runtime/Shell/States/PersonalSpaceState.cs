namespace ARGlasses.Interaction
{
    public enum PersonalSpaceState
    {
        Inactive,
        Active
    }

    public static class ExtensionPersonalSpaceState
    {
        public static bool IsActive(this PersonalSpaceState s) => s == PersonalSpaceState.Active;
        public static PersonalSpaceState Toggle(this PersonalSpaceState s) => s.IsActive() ? PersonalSpaceState.Inactive : PersonalSpaceState.Active;
    }
}
