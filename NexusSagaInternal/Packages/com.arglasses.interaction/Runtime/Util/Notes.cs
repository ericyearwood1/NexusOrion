using UnityEngine;

namespace ARGlasses.Interaction
{
    public class Notes : MonoBehaviour
    {
        [TextArea(3, 10), SerializeField] private string _body;
    }
}
