namespace ARGlasses.Components
{
    /// <summary>
    /// Defines the basic contract for a view in the AR Glasses architecture,
    /// allowing interaction with view models and controllers.
    /// </summary>
    /// <typeparam name="TViewModel">The type of the view model.</typeparam>
    /// <typeparam name="TInteractionModel">The type of the interaction model.</typeparam>
    public interface IArgView<TViewModel, TInteractionModel>
        where TViewModel : ViewModelBase
    {
        /// <summary>
        /// Initializes the view with the given interaction model (controller).
        /// </summary>
        /// <param name="controller">The interaction model to initialize the view.</param>
        void Initialize(TInteractionModel controller);

        /// <summary>
        /// Updates the view data based on the provided view model.
        /// </summary>
        /// <param name="viewModel">The view model containing the updated data.</param>
        void UpdateViewData(TViewModel viewModel);

        /// <summary>
        /// Sets the view's selection state for views that can toggle on/off.
        /// </summary>
        void Select(bool on);
    }
}
