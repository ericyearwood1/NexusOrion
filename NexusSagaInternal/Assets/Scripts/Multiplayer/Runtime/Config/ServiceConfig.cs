using UnityEngine;

namespace Multiplayer.Runtime.Config
{
    [CreateAssetMenu(menuName = "Nexus/Create service config", fileName = "SiroServiceConfig")]
    public class ServiceConfig : ScriptableObject
    {
        [SerializeField] private string _address = "0.0.0.0";
        [SerializeField] private string _editorAddress = "0.0.0.0";
        [SerializeField] private ushort _port = 5432;

        public string Address => _address;
        public string EditorAddress => _editorAddress;
        public ushort Port => _port;
    }
}