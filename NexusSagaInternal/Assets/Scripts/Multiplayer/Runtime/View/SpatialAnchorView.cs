using System.Collections.Generic;
using Multiplayer.Runtime.Data;
using UnityEngine;

namespace Multiplayer.Runtime.View
{
    public abstract class SpatialAnchorView : MonoBehaviour
    {
        public delegate void SharedAnchorEvent(SpatialAnchorView anchorView);
        public SharedAnchorEvent OnSaveSuccess;
        public SharedAnchorEvent OnSaveError;
        public SharedAnchorEvent OnShareError;
        public SharedAnchorEvent OnShareSuccess;
        
        [SerializeField] private GameObject _standardDebugView;
        [SerializeField] private GameObject _robotHomeDebugView;
        
        protected SpatialAnchorData _data;
        public int ShareRetries;
        public SpatialAnchorData Data => _data;
        public OVRSpatialAnchor.UnboundAnchor UnboundAnchor { get; set; }

        public abstract OVRSpatialAnchor GetOVRAnchor();
        
        public virtual void Initialise(SpatialAnchorData data, bool isShowDebugView)
        {
            _data = data;
            Debug.Log($"SpatialAnchorView::Initialise {data.AnchorType}");
            if (!isShowDebugView)
            {
                _robotHomeDebugView.SetActive(false);
                _standardDebugView.SetActive(false);
                return;
            }
            if (data.AnchorType == SpatialAnchorType.RobotHome)
            {
                _robotHomeDebugView.SetActive(true);
                _standardDebugView.SetActive(false);
            }
            else
            {
                _standardDebugView.SetActive(true);
                _robotHomeDebugView.SetActive(false);
            }   
        }

        public abstract bool IsLocalized();

        public abstract void Save();

        public abstract void Share(List<ulong> users);

        public bool IsSharedWithUser(ulong userId)
        {
            return Data.SharedWith.Contains(userId);
        }

        public abstract bool IsLoaded();

        public void CleanListeners()
        {
            OnShareSuccess = null;
            OnShareError = null;
        }
    }
}