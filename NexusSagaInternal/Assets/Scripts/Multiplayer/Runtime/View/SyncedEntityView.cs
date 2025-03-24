using UnityEngine;

namespace Multiplayer.Runtime.View
{
    public class SyncedEntityView : MonoBehaviour
    {
        private string _id;

        public string Id => _id;

        public void Initialise(string id)
        {
            _id = id;
        }
    }
}