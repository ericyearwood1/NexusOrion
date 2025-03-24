using TMPro;
using UnityEngine;


public class AppVersionDisplay : MonoBehaviour
{
    [SerializeField] private TMP_Text _text;

    private void Awake()
    {
        _text.text = $"v.{Application.version}";
    }
}
