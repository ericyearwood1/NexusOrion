using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ARGlassesRigHud : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ARGlassesRig _rig;
        [SerializeField, ReadOnly] private Selector _selector;
        private GUIStyle _default;
        private GUIStyle _green;
        private GUIStyle _red;
        private GUILayoutOption _noWrapStyle;

        private void Awake()
        {
            this.Scene(ref _rig);
            this.Scene(ref _selector);
            _default = new() { normal = new GUIStyleState() { textColor = Color.white }, wordWrap = false};
            _green = new() { normal = new GUIStyleState() { textColor = Color.green }, wordWrap = false};
            _red = new() { normal = new GUIStyleState() { textColor = Color.red }, wordWrap = false};
        }

        private void OnGUI()
        {
            try
            {
                _noWrapStyle = GUILayout.Width(Screen.width);
                GUILayout.BeginVertical();
                GUILayout.Label(WristbandConnected, _rig.Wristband.IsConnected ? _green : _red);
                GUILayout.Label(IMU, _default);
                GUILayout.Label(InputMode);
                GUILayout.Label(Surface, _noWrapStyle);
                GUILayout.Label(Hover, _noWrapStyle);
                GUILayout.Label(Selected, _noWrapStyle);
                GUILayout.EndVertical();
            }
            catch (Exception e)
            {
                Debug.LogError("This is crashing sometimes..?");
            }
        }

        private string Selected => $"Selected: {(_selector.Selected ? _selector.Selected.GetPath() : "None")}";
        private string Hover => $"Hover: {(_selector.Hovered ? _selector.Hovered.GetPath() : "None")}";
        private string Surface => $"Surface: {(_selector.TargetContext ? _selector.TargetContext.GetPath() : "None")}";
        private string InputMode => $"InputMode: {(_selector.InputContext ? _selector.InputContext.Name : "None")}";
        private string WristbandConnected => $"<b>Wristband</b>:{(_rig.Wristband.IsConnected ? "Connected" : "None")}";
        private string IMU => $"<color=yellow>IMU:</color>:{_rig.Wristband.ProjectedRadians}";

    }
}
