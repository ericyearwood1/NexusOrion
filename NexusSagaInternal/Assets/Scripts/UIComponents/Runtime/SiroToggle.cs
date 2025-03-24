using UnityEngine;
using UnityEngine.UI;

public class SiroToggle : Toggle
{
    [SerializeField] private Image _background;

    public Image Background => _background;
}
