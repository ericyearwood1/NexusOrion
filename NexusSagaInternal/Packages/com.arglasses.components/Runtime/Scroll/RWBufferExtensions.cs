using System.Collections.Generic;
using OSIG.Tools.Layout.Internals;

namespace ARGlasses.Components.Scroll
{
    public static class RWBufferExtensions
    {
        public static IEnumerable<T> AsEnumerable<T>(this RWBuffer<T> buffer, int length) where T : struct
        {
            for (int i = 0; i < length; i++)
            {
                yield return buffer[i];
            }
        }
    }
}
