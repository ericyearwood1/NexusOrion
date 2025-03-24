using FigmaUnity.FigmaComponents;
using UnityEngine;

namespace ARGlasses.Components
{
    public static class ArgComponentFactory
    {
        private static ComponentControllerCollection _collection;

        public static ComponentControllerCollection Collection
        {
            get
            {
                if (_collection == null)
                    _collection =
                        Resources.Load<ComponentControllerCollection>(ComponentControllerCollection.ResourcesPath);

                return _collection;
            }
        }


        public static ArgButton CreateARGButton(ButtonStyle style, ButtonViewModel viewModel, Transform parent = null)
        {
            var controller = Collection.InstantiateARGButton(parent);
            var styleSwitcher = controller.GetComponent<ButtonStyleEnforcer>();
            styleSwitcher.SetStyleAndPopulate(style);
            controller.ForceUpdateView();
            return controller;
        }

        public static ArgExpButton CreateARGExpButton(ExpButtonViewModel viewModel, Transform parent = null)
        {
            var controller = Collection.InstantiateARGExpButton(parent);
            controller.ForceUpdateView();
            return controller;
        }

        public static ArgAlertDialogue CreateARGAlertDialogue(AlertDialogueStyle style, AlertDialogueViewModel viewModel, Transform parent = null)
        {
            var controller = Collection.InstantiateARGAlertDialogue(parent);
            var styleSwitcher = controller.GetComponent<AlertDialogueStyleEnforcer>();
            styleSwitcher.SetStyleAndPopulate(style);
            controller.ViewModel = viewModel;
            controller.ForceUpdateView();
            return controller;
        }

        public static ArgAvatar CreateARGavatar(AvatarStyle style, AvatarViewModel viewModel, Transform parent = null)
        {
            var controller = Collection.InstantiateARGAvatar(parent);
            controller.Style = style;
            controller.ViewModel = viewModel;
            controller.ForceUpdateView();
            return controller;
        }

        public static ArgListItem CreateARGListItem(ListItemStyle style, ListItemViewModel viewModel, Transform parent = null)
        {
            var controller = Collection.InstantiateARGListItem(parent);
            var styleSwitcher = controller.GetComponent<ListItemStyleEnforcer>();
            styleSwitcher.SetStyleAndPopulate(style);
            controller.ForceUpdateView();
            return controller;
        }

        public static ArgTooltip CreateARGTooltip(TooltipStyle style, TooltipViewModel viewModel, Transform parent = null)
        {
            var controller = Collection.InstantiateARGTooltip(parent);
            var styleSwitcher = controller.GetComponent<TooltipStyleEnforcer>();
            controller.ViewModel = viewModel;
            styleSwitcher.SetStyleAndPopulate(style);
            controller.ForceUpdateView();
            return controller;
        }
    }
}
