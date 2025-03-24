using UnityEngine;

namespace ARGlasses.Components
{
    [RequireComponent(typeof(ArgAlertDialogue))]
    public class AlertDialogueStyleEnforcer : StyleEnforcerBase<ArgAlertDialogue, AlertDialogueViewModel>
    {
        [SerializeField] private AlertDialogueStyle _style;

        private AlertDialogueView _view;

        public void SetStyleAndPopulate(AlertDialogueStyle style)
        {
            _style = style;
            PopulatePrefab();
        }

        protected override void PopulateUse()
        {
            _view = _controller.GetComponentInChildren<AlertDialogueView>();
            _view.UpdateViewStyle(_style);
        }

        protected override GameObject GetPrefab()
        {
            return ViewCollectionManager.AlertDialogueCollection.TryGetPrefabForStyle(_style, out var prefab)
                ? prefab
                : null;
        }
    }
}
