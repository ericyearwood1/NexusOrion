using System.Linq;
using UnityEngine;

namespace Speech.Runtime
{
    public static class SilenceChecker
    {
        public static bool IsSilent(AudioClip audio, float threshold = 0.01f)
        {
            if (Application.isEditor)
            {
                return false;
            }
            if (audio== null)
            {
                return true;
            }
            var samples = new float[audio.samples * audio.channels];
            audio.GetData(samples, 0);
            var sum = samples.Sum(Mathf.Abs);
            var rms = Mathf.Sqrt(sum / samples.Length);
            Debug.Log($"RMS: {rms} Threshold: {threshold}");
            return rms < threshold;
        }
    }
}
