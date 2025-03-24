using CTRL.Schemes;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class ExtensionsDPad
    {
        public static bool IsNone(this DPad dPad) => dPad == DPad.None;
        public static bool IsUp(this DPad dPad) => dPad == DPad.Up;
        public static bool IsRight(this DPad dPad) => dPad == DPad.Right;
        public static bool IsDown(this DPad dPad) => dPad == DPad.Down;
        public static bool IsLeft(this DPad dPad) => dPad == DPad.Left;
        public static bool HasValue(this DPad dPad) => !dPad.IsNone();

        public static DPad ToDPad(this DPadAction action)
        {
            if (action.action == DPadAction.Action.Up) return DPad.Up;
            if (action.action == DPadAction.Action.Right) return DPad.Right;
            if (action.action == DPadAction.Action.Down) return DPad.Down;
            if (action.action == DPadAction.Action.Left) return DPad.Left;
            return DPad.None;
        }

        public static Vector3 ToVector3(this DPad dPad) => dPad.ToVector2();
        public static Vector2 ToVector2(this DPad dPad)
        {
            var dir = Vector2.zero;
            if (dPad.IsUp()) dir = Vector2.up;
            if (dPad.IsRight()) dir = Vector2.right;
            if (dPad.IsDown()) dir = Vector2.down;
            if (dPad.IsLeft()) dir = Vector2.left;
            return dir;
        }
    }

    public enum DPad
    {
        None,
        Up,
        Right,
        Down,
        Left
    }
}
