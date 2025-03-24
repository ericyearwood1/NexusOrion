using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(AudioSource))]
public class AudioVisualiser : MonoBehaviour
{
    public List<Transform> boxes;
    [Min(0f)] public float MinHeight = 10f;
    [Min(0f)] public float MaxHeight = 50f;
    [Range(64, 8192)] public int fftSize = 1024;
    [Range(0f, 1f)] public float smoothSpeed = 0.1f;

    private AudioSource _audioSource;
    private float[] _previousSpectrum;

    private bool _isActive;

    private void Awake()
    {
        _audioSource = gameObject.GetComponent<AudioSource>();
    }
    
    private void Start()
    {
        if (Microphone.devices.Length > 0)
        {
            var selectedMic = Microphone.devices[0];
            _audioSource.clip = Microphone.Start(selectedMic, true, 1, AudioSettings.outputSampleRate);
            _audioSource.loop = true;
            _audioSource.mute = true;
            while (!(Microphone.GetPosition(selectedMic) > 0)) {}
            _audioSource.Play();
            _previousSpectrum = new float[fftSize];
        }
        else Debug.LogError("No microphone detected!");
    }
    
    

    private void Update()
    {
        if (!_isActive) return;
        if (_audioSource == null || !_audioSource.isPlaying) return;
        
        var currentSpectrum = new float[fftSize];
        _audioSource.GetSpectrumData(currentSpectrum, 0, FFTWindow.Hamming);

        for (var i = 0; i < boxes.Count && i < currentSpectrum.Length; i++)
        {
            var smoothedValue = Mathf.Lerp(_previousSpectrum[i], currentSpectrum[i], smoothSpeed);
            var height = Mathf.Lerp(MinHeight, MaxHeight, smoothedValue * 100f);

            var newScale = boxes[i].localScale;
            newScale.y = height;
            boxes[i].localScale = newScale;

            _previousSpectrum[i] = smoothedValue;
        }
    }
    
    public void Show()
    {
        foreach (var box in boxes)
        {
            box.gameObject.SetActive(true);
        }
        _isActive = true;
    }
    
    public void Hide()
    {
        foreach (var box in boxes)
        {
            box.gameObject.SetActive(false);
        }
        _isActive = false;
    }
}