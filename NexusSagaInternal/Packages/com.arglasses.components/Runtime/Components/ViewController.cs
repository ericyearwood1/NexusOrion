using System;
using System.ComponentModel;
using ARGlasses.Interaction;
using UnityEngine;
using UnityEngine.Serialization;
using Component = UnityEngine.Component;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace ARGlasses.Components
{
    /// <summary>
    /// Serves as the base class for view controllers, responsible for managing the
    /// lifecycle and state of view components.
    /// </summary>
    public abstract class ViewController : MonoBehaviour
    {
        // Event fired once initialization is complete
        private event Action WhenInitialized = delegate {  };

        // Flag to ensure one-time initialization
        private bool _initializationEnsured = false;

        // Event fired upon object destruction
        private event Action WhenDestroyed = delegate {  };

        // Flag to track if the object has been destroyed
        private bool _destroyed;

        /// <summary>
        /// Ensures initialization is completed once and fires InitializationEnsured event.
        /// </summary>
        private void EnsureInitialization()
        {
            if (_initializationEnsured) return;
            _initializationEnsured = true;
            WhenInitialized();
        }

        public void Subscribe(Action initialized, Action destroyed)
        {
            SubscribeToInitialization(initialized);
            SubscribeToDestruction(destroyed);
        }

        /// <summary>
        /// Subscribes a callback to the InitializationEnsured event. Executes
        /// immediately if already initialized.
        /// </summary>
        /// <param name="callback">The method to call on initialization.</param>
        public void SubscribeToInitialization(Action callback)
        {
            if (_initializationEnsured) callback();
            else WhenInitialized += callback;
        }

        /// <summary>
        /// Subscribes a callback to the OnDestroyed event. Executes immediately
        /// if already destroyed.
        /// </summary>
        /// <param name="callback">The method to call on destruction.</param>
        public void SubscribeToDestruction(Action callback)
        {
            if (_destroyed) callback();
            else WhenDestroyed += callback;
        }

        /// <summary>
        /// Unity's OnDestroy method override to manage custom destruction logic.
        /// </summary>
        private void OnDestroy()
        {
            if (_destroyed) return;

            _destroyed = true;
            WhenDestroyed();
        }

        /// <summary>
        /// Forces an update of the view without relying on a model.
        /// </summary>
        public abstract void ForceUpdateViewNoModel();

        /// <summary>
        /// Initialize component-related elements.
        /// </summary>
        /// <param name="ensure">Whether to perform initialization checks.</param>
        protected virtual void InitializeComponents(bool ensure)
        {
            if (!ensure) return; // TODO: Consider not calling in this case?

            InitializeCursorEffect();
            EnsureInitialization();
        }

        /// <summary>
        /// Initializes the cursor effect based on related components.
        /// </summary>
        private void InitializeCursorEffect()
        {
            this.Ensure<ArgCursorEffect>();
        }
    }

    public abstract class ViewController<TViewModel, TView> : ViewController
        where TViewModel : ViewModelBase, new()
        where TView : IArgView<TViewModel, ViewController>
    {
        protected ComponentViewRootProvider ComponentViewRoot;
        [HideInInspector] public TView View;
        [FormerlySerializedAs("_viewModel")] public TViewModel ViewModel = new TViewModel();

        public ViewController()
        {
            ViewModel.PropertyChanged += ViewModel_PropertyChanged;
        }

        private void ViewModel_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            ForceUpdateView();
        }

        protected override void InitializeComponents(bool ensure)
        {
            ComponentViewRoot = GetComponentInChildren<ComponentViewRootProvider>();

            base.InitializeComponents(ensure);
        }

        public virtual void ForceUpdateView()
        {
            try
            {
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
                InitializeComponents(ensure: false);
                View = GetComponentInChildren<TView>();

                if (View == null || ViewModel == null) return;
                View.UpdateViewData(ViewModel);
            }
            catch (Exception e)
            {
                Debug.Log(e);
                throw;
            }
        }

        public override void ForceUpdateViewNoModel()
        {
            ForceUpdateView();
        }
    }

    public abstract class ViewController<TViewModel, TInteractionModel, TView> : ViewController
        where TViewModel : ViewModelBase, new()
        where TInteractionModel : Component
        where TView : IArgView<TViewModel, TInteractionModel>
    {
        private ComponentViewRootProvider _componentViewRoot;
        public ComponentViewRootProvider ComponentViewRoot => _componentViewRoot;

        private TInteractionModel _interactionModel;
        public TInteractionModel InteractionModel => _interactionModel;

        [FormerlySerializedAs("_viewModel")] public TViewModel ViewModel = new TViewModel();
        [HideInInspector] public TView View;

        public ViewController()
        {
            ViewModel.PropertyChanged += ViewModel_PropertyChanged;
        }

        private void ViewModel_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            ForceUpdateView();
        }

        protected override void InitializeComponents(bool ensure)
        {
            if (ensure)
            {
                this.Ensure(ref _interactionModel, relation: Relation.Descendant);
                this.Descendant(ref _componentViewRoot);
            }
            else
            {
                this.Descendant(ref _interactionModel, optional: true);
                this.Descendant(ref _componentViewRoot, optional: true, searchOrder: SearchOrder.FirstInHierarchy);
            }

            base.InitializeComponents(ensure);
        }

        public virtual void ForceUpdateView()
        {
            try
            {
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
                InitializeComponents(ensure: false);
                View = GetComponentInChildren<TView>();

                if (View == null || ViewModel == null) return;
                View.UpdateViewData(ViewModel);
            }
            catch (Exception e)
            {
                Debug.Log(e);
                throw;
            }
        }

        public override void ForceUpdateViewNoModel()
        {
            ForceUpdateView();
        }
    }
}
