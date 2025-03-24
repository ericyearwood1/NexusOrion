using UnityEngine;

namespace ARGlasses.Interaction
{
    public enum DiscreteHandMotion
    {
        None,
        Push,
        Pull
    }

    public static class ExtensionsDiscreteHandMotion
    {
        public static bool IsNone(this DiscreteHandMotion motion) => motion == DiscreteHandMotion.None;
        public static bool IsPush(this DiscreteHandMotion motion) => motion == DiscreteHandMotion.Push;
        public static bool IsPull(this DiscreteHandMotion motion) => motion == DiscreteHandMotion.Pull;

        public static Vector3 ToVector3(this DiscreteHandMotion motion)
        {
            if (motion.IsPush()) return Vector3.forward;
            if (motion.IsPull()) return Vector3.back;
            return Vector3.zero;
        }
    }
}
