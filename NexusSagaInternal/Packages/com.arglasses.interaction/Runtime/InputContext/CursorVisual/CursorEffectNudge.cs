using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Obsolete("ICursorVisualEffect implements nudging now")]
    public class CursorEffectNudge : MonoBehaviour
    {
        private void Awake() => Debug.LogWarning("Obsolete.  ICursorVisualEffect implements nudging now.");
    }
}
