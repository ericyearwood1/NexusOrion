using System.Collections.Generic;
using Notifications.Runtime.Data;
using Notifications.Runtime.View;
using UnityEngine;

namespace Notifications.Runtime.Systems
{
    public class NotificationSystem
    {
        private readonly Transform _container;
        private readonly Queue<SiroNotificationView> _pool;
        private readonly List<SiroNotificationView> _activeViews;

        public NotificationSystem(int poolSize, Transform container, GameObject prefab)
        {
            _container = container;
            _pool = new Queue<SiroNotificationView>(poolSize);
            _activeViews = new List<SiroNotificationView>(poolSize);
            for (var i = 0; i < poolSize; ++i)
            {
                var notificationGO = GameObject.Instantiate(prefab, container, false);
                notificationGO.transform.localPosition = Vector3.zero;
                notificationGO.transform.localScale = Vector3.one;
                notificationGO.transform.localRotation = Quaternion.identity;
                _pool.Enqueue(notificationGO.GetComponent<SiroNotificationView>());
                notificationGO.SetActive(false);
            }
        }

        private bool _isEnabled;

        public void SetEnabled(bool isEnabled)
        {
            _isEnabled = isEnabled;
        }
        
        public void ShowNotification(string text, NotificationType type)
        {
            if (!_isEnabled) return;
            var view = _pool.Dequeue();
            var viewTransform = view.transform;
            viewTransform.SetParent(_container);
            viewTransform.localPosition = Vector3.zero;
            viewTransform.localScale = Vector3.one;
            viewTransform.localEulerAngles = Vector3.zero;
            view.Initialise(text, type);
            foreach (var activeView in _activeViews)
            {
                activeView.SetDisplayTime(1);
            }
            _activeViews.Add(view);
            view.gameObject.SetActive(true);
        }

        public void Tick()
        {
            for (var i = 0; i < _activeViews.Count; ++i)
            {
                var activeView = _activeViews[i];
                if (activeView.State != ViewState.Hidden) continue;
                activeView.gameObject.SetActive(false);
                activeView.Reset();
                _pool.Enqueue(activeView);
                _activeViews.RemoveAt(i);
            }
        }
    }
}