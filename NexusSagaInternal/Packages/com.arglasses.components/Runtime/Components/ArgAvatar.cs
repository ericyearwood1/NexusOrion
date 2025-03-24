using ARGlasses.Interaction;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;
using UnityEngine.Events;

namespace ARGlasses.Components
{
    public class ArgAvatar : ViewController<AvatarViewModel, ButtonModel, AvatarView>
    {
        [Tooltip(TooltipMessages.AutoPopulateMessage)] [SerializeField, ReadOnly]
        private ComponentViewRootProvider _componentViewRoot;

        public UnityEvent WhenClick;


        private void Start()
        {
            InitializeComponents(ensure:true);
            ForceUpdateView();
            View.Initialize(InteractionModel);
            InteractionModel.WhenClicked += Clicked;
        }

        private void Clicked()
        {
            WhenClick?.Invoke();
        }

        private void OnDestroy()
        {
            InteractionModel.WhenClicked -= Clicked;
        }

        protected void OnValidate()
        {
            ForceUpdateView();
        }

        public void PopulatePrefab()
        {
            InitializeComponents(ensure:false);
            if (ViewCollectionManager.AvatarCollection.TryGetPrefabForStyle(ViewModel.Style, out var prefab))
            {
                DestroyAllChildren(_componentViewRoot.transform);
#if UNITY_EDITOR
                var instantiatePrefab =
                    (GameObject)PrefabUtility.InstantiatePrefab(prefab, _componentViewRoot.transform);
#else
                var instantiatePrefab =
                    Instantiate(prefab, _componentViewRoot.transform);
#endif
                ForceUpdateView();
            }
            else
            {
                Debug.LogWarning($"Prefab for style {ViewModel.Style} not found.");
            }
        }

        private void DestroyAllChildren(Transform parent)
        {
            while (parent.childCount > 0)
            {
                var child = parent.GetChild(0);
                DestroyImmediate(child.gameObject);
            }
        }

        public Sprite AvatarImage
        {
            get => ViewModel.AvatarImage;
            set
            {
                ViewModel.AvatarImage = value;
                ForceUpdateView();
            }
        }

        public Sprite AppImage
        {
            get => ViewModel.AppImage;
            set
            {
                ViewModel.AppImage = value;
                ForceUpdateView();
            }
        }

        public string LabelText
        {
            get => ViewModel.LabelText;
            set
            {
                ViewModel.LabelText = value;
                ForceUpdateView();
            }
        }

        public string IndicatorText
        {
            get => ViewModel.CounterText;
            set
            {
                ViewModel.CounterText = value;
                ForceUpdateView();
            }
        }

        public bool ShowLabelOnHover
        {
            get => ViewModel.ShowLabelOnHover;
            set
            {
                ViewModel.ShowLabelOnHover = value;
                ForceUpdateView();
            }
        }

        public AvatarStyle Style
        {
            get => ViewModel.Style;
            set
            {
                ViewModel.Style = value;
                PopulatePrefab();
            }
        }
    }

    public enum AvatarSize
    {
        Small,
        Large
    }
}
