// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using UnityEditor;
using UnityEngine;

namespace ARGlasses.Interaction
{
    // Make sure not to strip this from AppX builds or else it'll fail
    public class CtrlProcess : MonoBehaviour
    {
        public const string AssetName = nameof(CtrlProcess);
        private const string MenuName = "OrionIX/" + AssetName;
        private const string Src2PathKey = "OrionIX.Src2Path";

        public static string Src2Path
        {
#if UNITY_EDITOR
            get => EditorPrefs.GetString(Src2PathKey, ".../src2");
            set => EditorPrefs.SetString(Src2PathKey, value);
#else
            get => ".../src2";
            set { }
#endif
        }

        public string Src2Branch = "master";

        public string Src2Yaml =
            "platform/ctrl-r/pipeline_configs/experimental/interaction_frontier/modules/complete/sc_cc_complete.yaml";

        // [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterAssembliesLoaded)]
        // static void Init() { }
    }
}
