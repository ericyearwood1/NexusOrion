using System.Collections.Generic;
using Robot.Runtime.Data;
using Robot.Runtime.Data.Robot;
using Robot.Runtime.View;
using UnityEngine;

namespace Robot.Runtime.Systems
{
    /// <summary>
    /// A system responsible for managing highlight markers in a 3D environment. 
    /// This system uses a pool of highlight views that can be instantiated and reused to visually highlight objects 
    /// based on the provided data. Highlights are displayed at specific positions in the world, and they can be 
    /// hidden or reset based on specific events or actions. The system can be enabled or disabled dynamically.
    /// </summary>
    /// <typeparam name="V">The type of the highlight view (must inherit from <see cref="HighlightView"/>).</typeparam>
    /// <typeparam name="D">The type of data that represents the highlight information (must inherit from <see cref="HighlightData"/>).</typeparam>
    public class HighlightSystem<V, D> where V : HighlightView where D : HighlightData 
    {
        // private const int POOL_SIZE = 20;
        private Queue<V> _pool;
        private List<V> _activeItems = new ();
        protected V _currentView;
        protected Transform _cameraTransform;
        private bool _isEnabled;

        public void Initialise(Transform cameraTransform, Transform highlightContainer, GameObject highlightPrefab,
            Color color,int poolSize)
        {
            _isEnabled = true;
            _cameraTransform = cameraTransform;
            InitialisePool(highlightContainer, highlightPrefab, color, poolSize);
        }

        public void SetEnabled(bool isEnabled)
        {
            _isEnabled = isEnabled;
            if (!_isEnabled)
            {
                HideAllMarkers();
            }
        }

        private void InitialisePool(Transform highlightContainer, GameObject highlightPrefab, Color color, int poolSize)
        {
            _pool = new Queue<V>(poolSize);
            for (var i = 0; i < poolSize; ++i)
            {
                var prefab = Object.Instantiate(highlightPrefab, highlightContainer, true);
                var highlightView = prefab.GetComponent<V>();
                highlightView.Reset();
                highlightView.SetColor(color);
                _pool.Enqueue(highlightView);
            }
        }

        public virtual void OnHighlightReceived(D data)
        {
            if (!_isEnabled) return;
            _currentView = _pool.Dequeue();
            _currentView.transform.localPosition = data.Position;
            _activeItems.Add(_currentView);
            InitialiseView(data);
        }

        protected virtual void InitialiseView(D data)
        {
        }
        
        public void HideAllMarkers()
        {
            for (var i = _activeItems.Count - 1; i >= 0; --i)
            {
                var view = _activeItems[i];
                view.Hide();
            }
        }

        public void OnInstructionComplete()
        {
            for (var i = _activeItems.Count - 1; i >= 0; --i)
            {
                var view = _activeItems[i];
                view.OnInstructionComplete();
            }
            _currentView = null;
        }
        
        public void OnActionComplete()
        {
            for (var i = _activeItems.Count - 1; i >= 0; --i)
            {
                var view = _activeItems[i];
                view.OnActionComplete();
            }
            _currentView = null;
        }

        public virtual void Tick()
        {
            for (var i = _activeItems.Count - 1; i >= 0; --i)
            {
                var view = _activeItems[i];
                if (view.State != HighlightState.Hidden) continue;
                view.Reset();
                view.Dispose();
                _activeItems.RemoveAt(i);
                _pool.Enqueue(view);
            }
        }
    }
}