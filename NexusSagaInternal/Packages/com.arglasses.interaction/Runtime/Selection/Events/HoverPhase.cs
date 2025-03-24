using static ARGlasses.Interaction.HoverPhase;

namespace ARGlasses.Interaction
{
    public static class HoverPhaseExtensions
    {
        public static bool IsNone(this HoverPhase phase) => phase == None;
        public static bool IsBegin(this HoverPhase phase) => phase == Begin;
        public static bool IsUpdate(this HoverPhase phase) => phase == Update;
        public static bool IsEnd(this HoverPhase phase) => phase == End;
    }

    public enum HoverPhase
    {
        None,
        Begin,
        Update,
        End,
    }
}
