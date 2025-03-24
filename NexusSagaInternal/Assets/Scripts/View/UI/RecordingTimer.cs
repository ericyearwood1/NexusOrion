using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RecordingTimer : MonoBehaviour
{
    public Slider timerSlider;
    
    public void SetValue(float value)
    {
        timerSlider.value = value;
    }
}
