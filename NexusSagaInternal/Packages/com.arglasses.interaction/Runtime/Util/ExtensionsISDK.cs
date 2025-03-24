#if ISDK_PRESENT
using Oculus.Interaction;
using Oculus.Interaction.Surfaces;
#endif
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class ExtensionsISDK
    {
#if ISDK_PRESENT
        public static TargetState GetState(this InteractableState state)
        {
            if (state.IsNormal()) return TargetState.Normal;
            if (state.IsHover()) return TargetState.Hover;
            if (state.IsSelect()) return TargetState.Press;
            if (state.IsDisabled()) return TargetState.Disabled;
            return TargetState.Normal;
        }

        public static bool IsLeft(this Oculus.Interaction.Input.Handedness h) => h == Oculus.Interaction.Input.Handedness.Left;
        public static bool IsRight(this Oculus.Interaction.Input.Handedness h) => h == Oculus.Interaction.Input.Handedness.Right;


        public static bool IsHover(this PointerEvent evt) => evt.Type == PointerEventType.Move;
        public static bool IsUnhover(this PointerEvent evt) => evt.Type == PointerEventType.Unhover;
        public static bool IsSelect(this PointerEvent evt) => evt.Type == PointerEventType.Select;
        public static bool IsUnselect(this PointerEvent evt) => evt.Type == PointerEventType.Unselect;
        public static bool IsMove(this PointerEvent evt) => evt.Type == PointerEventType.Move;
        public static bool IsCancel(this PointerEvent evt) => evt.Type == PointerEventType.Cancel;

        public static PointerEvent WithType(this PointerEvent evt, PointerEventType t) => new(evt.Identifier, t, evt.Pose, evt.Data);

        public static Pose DebugGizmosAxis(this Pose p, float size = 1.0f)
        {
            DebugGizmos.DrawAxis(p, size);
            return p;
        }

        public static Pose DebugGizmosAxis(this Pose p, Color c, float size = 1.0f)
        {
            DebugGizmos.Color = c;
            DebugGizmos.DrawAxis(p, size);
            return p;
        }

        public static bool IsNormal(this InteractableState state) => state == InteractableState.Normal;
        public static bool IsHover(this InteractableState state) => state == InteractableState.Hover;
        public static bool IsSelect(this InteractableState state) => state == InteractableState.Select;
        public static bool IsDisabled(this InteractableState state) => state == InteractableState.Disabled;

        public static TargetState ToTargetState(this InteractableState state)
        {
            if (state == InteractableState.Hover) return TargetState.Hover;
            if (state == InteractableState.Select) return TargetState.Press;
            if (state == InteractableState.Disabled) return TargetState.Disabled;
            return TargetState.Normal;
        }

        public static Side ToSide(this Oculus.Interaction.Input.Handedness h) => h.IsLeft() ? Side.Left : Side.Right;
        public static bool Equals(this Oculus.Interaction.Input.Handedness h, Side s) => h.IsLeft() && s.IsLeft() || h.IsRight() && s.IsRight();
        public static bool Equals(this Side s, Oculus.Interaction.Input.Handedness h) => h.Equals(s);

        public static Vector3 ClosestPoint(this ISurfacePatch surfacePatch, Vector3 p)
        {
            surfacePatch.ClosestSurfacePoint(p, out var closesSurfaceHit);
            return closesSurfaceHit.Point;
        }
#endif
    }
}
