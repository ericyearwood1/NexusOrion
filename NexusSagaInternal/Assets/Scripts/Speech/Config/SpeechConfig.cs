using UnityEngine;

namespace Speech.Config
{
    [CreateAssetMenu(menuName = "Nexus/Create speech config", fileName = "SpeechConfig")]
    public class SpeechConfig : ScriptableObject
    {
        [SerializeField] private string _address = "0.0.0.0";
        [SerializeField] private string _editorAddress = "0.0.0.0";
        [SerializeField] private ushort _port = 5000;
        
        
        public string Address => _address;
        public string EditorAddress => _editorAddress;
        public ushort Port => _port;
        
    }
}
