using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Multiplayer.Runtime.Data;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Multiplayer.Runtime.View
{
    public class StubSpatialAnchorView : SpatialAnchorView
    {
        private int _maxTicks = 30;
        private int _currentLocalizedTicks;

        public override void Initialise(SpatialAnchorData data, bool isShowDebugView)
        {
            base.Initialise(data, isShowDebugView);
            Debug.Log($"Initialise {_data.AnchorType} | {_currentLocalizedTicks}");
            _data.UUID = Guid.NewGuid();
            _currentLocalizedTicks = 0;
            _maxTicks = Random.Range(0, 30);
        }

        //@TODO get rid of this
        public override OVRSpatialAnchor GetOVRAnchor()
        {
            return null;
        }
        
        public override bool IsLocalized()
        {
            if (_currentLocalizedTicks >= _maxTicks)
            {
                return true;
            }
            _currentLocalizedTicks++;
            return false;
        }

        public override void Save()
        {
            DoSave();
        }

        private async Task DoSave()
        {
            await Task.Delay(1000);
            OnSaveSuccess?.Invoke(this);
        }

        public override void Share(List<ulong> users)
        {
            foreach (var user in users)
            {
                _data.SharedWith.Add(user);
            }
            DoShare();
        }

        public override bool IsLoaded()
        {
            return IsLocalized();
        }

        private async Task DoShare()
        {
            await Task.Delay(1000);
            OnShareSuccess?.Invoke(this);
        }

    }
}