using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class CTRLBluetoothConnection : MonoBehaviour
    {
        [SerializeField, ReadOnly] private bool _emgPassthroughLaunchSuccess;
        public bool EmgPassthroughLaunchSuccess => _emgPassthroughLaunchSuccess;
        public event Action<bool> WhenEmgPassthroughLaunchComplete = delegate { };

        private void Awake()
        {
            if (!isActiveAndEnabled) return;

            // this code was used to auto-launch the WearableInputService passthrough functionality, and is not in use atm
#if false && UNITY_ANDROID && !UNITY_EDITOR
            string failureAdvice = $"Confirm that you have com.meta.emg.client.app.passthrough installed on your headset and that your AndroidManifest.xml includes: " +
                                        $"<queries><package android:name=\"com.meta.emg.client.app.passthrough\"/></queries>";

            // Make sure you have added the following line to your AndroidManifest.xml:
            // <queries>
            //     <package android:name="com.meta.emg.client.app.passthrough"/>
            // </queries>

            var bundleId = "com.meta.emg.client.app.passthrough";

            try
            {
                using AndroidJavaClass unityPlayer = new AndroidJavaClass("com.unity3d.player.UnityPlayer");
                using AndroidJavaObject currentActivity = unityPlayer.GetStatic<AndroidJavaObject>("currentActivity");
                using AndroidJavaObject packageManager = currentActivity.Call<AndroidJavaObject>("getPackageManager");
                using AndroidJavaObject launchIntent = packageManager.Call<AndroidJavaObject>("getLaunchIntentForPackage", bundleId);

                Debug.Log($"Attempting to launch {bundleId}");
                currentActivity.Call("startActivity", launchIntent);
                Debug.Log($"Success launching {bundleId}");
                _emgPassthroughLaunchSuccess = true;
            }
            catch (Exception e)
            {
                Debug.LogError($"{nameof(CTRLBluetoothConnection)} failed when attempting to launch {bundleId}");
                Debug.LogError(failureAdvice);

                Debug.LogException(e);
                _emgPassthroughLaunchSuccess = false;
            }
            finally
            {
                WhenEmgPassthroughLaunchComplete(_emgPassthroughLaunchSuccess);
            }
#endif
        }
    }
}
