// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using CTRL;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class CTRLMacEditorDisabler : MonoBehaviour
    {
#if UNITY_EDITOR_OSX
        [RuntimeInitializeOnLoadMethod]
        public static void DisableCTRL()
        {
            Debug.Log("Disabling CTRL functionality in Mac Editor");
            CTRLClient client = FindObjectOfType<CTRLClient>();
            if(client) client.enabled = false;
        }
#endif
    }
}
