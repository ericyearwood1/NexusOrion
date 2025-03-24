using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace ARGlasses.Components
{
    /// <summary>
    /// Serves as the base class for view models, offering property-change notification
    /// to support data binding.
    /// </summary>
    [Serializable]
    public class ViewModelBase : INotifyPropertyChanged
    {
        /// <summary>
        /// Event triggered when a property changes.
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Notifies subscribers that a property has changed.
        /// </summary>
        /// <param name="propertyName">The name of the property that changed.</param>
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        /// <summary>
        /// Sets the value of a field and notifies subscribers if it changes.
        /// </summary>
        protected bool SetField<T>(ref T field, T value, [CallerMemberName] string propertyName = null)
        {
            // Check if new value is the same as old value
            if (EqualityComparer<T>.Default.Equals(field, value)) return false;

            // Update the field and trigger the PropertyChanged event
            field = value;
            OnPropertyChanged(propertyName);
            return true;
        }
    }
}
