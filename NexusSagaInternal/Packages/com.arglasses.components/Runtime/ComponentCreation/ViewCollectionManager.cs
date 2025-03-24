using UnityEngine;

namespace ARGlasses.Components
{
    public static class ViewCollectionManager
    {
        private static ButtonStylePrefabCollection _buttonCollection;
        private static ButtonUseStateMachineCollection _buttonStateMachineCollection;
        private static TileButtonStylePrefabCollection _tileButtonCollection;
        private static AvatarStylePrefabCollection _avatarCollection;
        private static AlertDialoguePrefabCollection _alertDialogueCollection;
        private static ListItemStylePrefabCollection _listItemCollection;
        private static SliderStylePrefabCollection _sliderStyleCollection;
        private static ToggleStylePrefabCollection _toggleStyleCollection;
        private static TooltipStylePrefabCollection _tooltipStyleCollection;
        private static TyperampCollection _typerampCollection;
        private static OverflowMenuCollection _overflowMenuCollection;

        public static void ResetResources()
        {
            _buttonCollection = null;
            _buttonStateMachineCollection = null;
            _tileButtonCollection = null;
            _avatarCollection = null;
            _alertDialogueCollection = null;
            _listItemCollection = null;
            _sliderStyleCollection = null;
            _toggleStyleCollection = null;
            _tooltipStyleCollection = null;
            _typerampCollection = null;
            _overflowMenuCollection = null;
        }

        public static ButtonStylePrefabCollection ButtonCollection
        {
            get
            {
                if (_buttonCollection == null)
                {
                    _buttonCollection =
                        Resources.Load<ButtonStylePrefabCollection>(ButtonStylePrefabCollection.ResourcesPath);
                }

                return _buttonCollection;
            }
        }

        public static AlertDialoguePrefabCollection AlertDialogueCollection
        {
            get
            {
                if (_alertDialogueCollection == null)
                {
                    _alertDialogueCollection =
                        Resources.Load<AlertDialoguePrefabCollection>(AlertDialoguePrefabCollection.ResourcesPath);
                }

                return _alertDialogueCollection;
            }
        }

        public static ButtonUseStateMachineCollection ButtonStateMachineCollection
        {
            get
            {
                if (_buttonStateMachineCollection == null)
                {
                    _buttonStateMachineCollection =
                        Resources.Load<ButtonUseStateMachineCollection>(ButtonUseStateMachineCollection.ResourcesPath);
                }

                return _buttonStateMachineCollection;
            }
        }

        public static TileButtonStylePrefabCollection TileButtonCollection
        {
            get
            {
                if (_tileButtonCollection == null)
                {
                    _tileButtonCollection =
                        Resources.Load<TileButtonStylePrefabCollection>(TileButtonStylePrefabCollection.ResourcesPath);
                }

                return _tileButtonCollection;
            }
        }

        public static AvatarStylePrefabCollection AvatarCollection
        {
            get
            {
                if (_avatarCollection == null)
                {
                    _avatarCollection =
                        Resources.Load<AvatarStylePrefabCollection>(AvatarStylePrefabCollection.ResourcesPath);
                }

                return _avatarCollection;
            }
        }

        public static ListItemStylePrefabCollection ListItemCollection
        {
            get
            {
                if (_listItemCollection == null)
                {
                    _listItemCollection =
                        Resources.Load<ListItemStylePrefabCollection>(ListItemStylePrefabCollection.ResourcesPath);
                }

                return _listItemCollection;
            }
        }

        public static SliderStylePrefabCollection SliderStyleCollection
        {
            get
            {
                if (_sliderStyleCollection == null)
                {
                    _sliderStyleCollection =
                        Resources.Load<SliderStylePrefabCollection>(SliderStylePrefabCollection.ResourcesPath);
                }

                return _sliderStyleCollection;
            }
        }

        public static ToggleStylePrefabCollection ToggleStyleCollection
        {
            get
            {
                if (_toggleStyleCollection == null)
                {
                    _toggleStyleCollection =
                        Resources.Load<ToggleStylePrefabCollection>(ToggleStylePrefabCollection.ResourcesPath);
                }

                return _toggleStyleCollection;
            }
        }
        
        public static TooltipStylePrefabCollection TooltipStyleCollection
        {
            get
            {
                if (_tooltipStyleCollection == null)
                {
                    _tooltipStyleCollection =
                        Resources.Load<TooltipStylePrefabCollection>(TooltipStylePrefabCollection.ResourcesPath);
                }

                return _tooltipStyleCollection;
            }
        }

        public static TyperampCollection TyperampCollection
        {
            get
            {
                if (_typerampCollection == null)
                {
                    _typerampCollection =
                        Resources.Load<TyperampCollection>(TyperampCollection.ResourcesPath);
                }

                return _typerampCollection;
            }
        }
        
        public static OverflowMenuCollection OverflowMenuCollection
        {
            get
            {
                if (_overflowMenuCollection == null)
                {
                    _overflowMenuCollection =
                        Resources.Load<OverflowMenuCollection>(OverflowMenuCollection.ResourcesPath);
                }

                return _overflowMenuCollection;
            }
        }
    }
}
