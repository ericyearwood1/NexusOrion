using UnityEngine;

namespace ARGlasses.Interaction
{
    public class SimpleDPadLogger : MonoBehaviour
    {
        [SerializeField] private Target _target;

        private void Awake() => this.Ensure(ref _target);
        private void OnEnable() => _target.WhenSelecting += HandleSelection;
        private void OnDisable() => _target.WhenSelecting -= HandleSelection;

        private void HandleSelection(ISelection selection)
        {
            // selection provides us with access to Current, Begin, and Previous states.
            // since DPad actions are discrete, we really only ever want to use 'Current'
            var dPad = selection.Latest.DPad;

            if (dPad.IsNone()) return;
            if(dPad.IsUp()) Debug.Log("Do something Up!");
            if(dPad.IsRight()) Debug.Log("Do something Right!");
            if(dPad.IsDown()) Debug.Log("Do something Down!");
            if(dPad.IsLeft()) Debug.Log("Do something Left!");
        }
    }
}
