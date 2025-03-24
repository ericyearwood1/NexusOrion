using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class ExtensionsSide
    {
        public static Side SideFromName(this Component component)
        {
            if (component.name.ToLower().Contains("left")) return Side.Left;
            if (component.name.ToLower().Contains("right")) return Side.Right;

            var parent = component.transform.parent;
            if (parent != null) return parent.SideFromName();

            Debug.LogError($"Could not find Side");
            return Side.None;
        }
        public static bool IsNone(this Side h) => h == Side.None;
        public static bool HasValue(this Side h) => h.IsLeft() || h.IsRight();

        public static bool IsLeft(this Side h) => h == Side.Left;
        public static bool IsRight(this Side h) => h == Side.Right;
        public static bool IsLeftOrRight(this Side h) => h.IsLeft() || h.IsRight();

        public static bool IsBoth(this Side h) => h == Side.Both;
    }

    public enum Side
    {
        None,
        Left,
        Right,
        Both,
    }
}
