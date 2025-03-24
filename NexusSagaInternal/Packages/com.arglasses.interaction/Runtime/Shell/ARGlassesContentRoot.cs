using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public class ARGlassesContentRoot : MonoBehaviour
    {
        [SerializeField, ReadOnly] private PersonalSpaceRoot _personalSpaceRoot;
        public PersonalSpaceRoot PersonalSpaceRoot => _personalSpaceRoot;

        [SerializeField, ReadOnly] private WorldSpaceRoot _worldSpaceRoot;
        public WorldSpaceRoot WorldSpaceRoot => _worldSpaceRoot;

        protected void Awake()
        {
            this.Descendant(ref _personalSpaceRoot);
            this.Descendant(ref _worldSpaceRoot);
        }
    }
}
