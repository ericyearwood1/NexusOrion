// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

namespace ARGlasses.Interaction
{
    public enum Chirality
    {
        Left,
        Right,
        COUNT,
        NONE = -1
    }

    public static class ExtensionsGazeChirality
    {
        public static bool IsLeft(this Chirality chirality)
        {
            return chirality == Chirality.Left;
        }

        public static bool IsRight(this Chirality chirality)
        {
            return chirality == Chirality.Right;
        }

        public static int ToId(this Chirality chirality)
        {
            return (int)chirality;
        }
    }
}
