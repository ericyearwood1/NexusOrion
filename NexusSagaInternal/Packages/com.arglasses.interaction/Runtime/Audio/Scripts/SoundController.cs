// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class SoundController : MonoBehaviour
    {
        public enum ESoundClip
        {
            None = 0,
            Select = 1,
            Back = 2,
            Close = 3,
            Notification = 4,
            ScrollDown = 5,
            ScrollUp = 6,
            ObjectPlaced = 7,
            LauncherIn = 8,
            LauncherOut = 9,
            WakeIn = 10,
            WakeOut = 11,
            Press = 12,
            Grab = 14,
            ContextMenuIn = 15,
            ContextMenuOut = 16,
            PressCVIn = 17,
            PressCVOut = 18,
            PressEMGIn = 19,
            PressEMGOut = 20,
            MessageNotification = 21,
            IncomingCallNotification = 22,
            ToggleOn = 23,
            ToggleOff = 24,
            Notch = 25,
        }

        private static SoundController _instance;

        public static SoundController Instance
        {
            get
            {
                if (!_instance) _instance = new GameObject("SoundController").AddComponent<SoundController>();
                return _instance;
            }
        }

        private AudioSource _audioSource;

        private void Awake()
        {
            _audioSource = gameObject.AddComponent<AudioSource>();
        }

        public const int PitchSelectedSemitones = 7;
        public const int PitchDriftInSemitones = -1;
        public const int PitchDriftOutSemitones = -3;
        public const int PitchCancelSemitones = -8;
        public void PlaySelected() => Play(ESoundClip.Grab, volumeScale: 0.4f, pitch: CalculateFrequencyRatio(PitchSelectedSemitones));
        public void PlayCancel() => Play(ESoundClip.Grab, volumeScale: 0.3f, pitch: CalculateFrequencyRatio(PitchCancelSemitones));
        public void PlayDriftOut() => Play(ESoundClip.Grab, volumeScale: 0.2f, pitch: CalculateFrequencyRatio(PitchDriftOutSemitones));
        public void PlayDriftIn() => Play(ESoundClip.Grab, volumeScale: 0.2f, pitch: CalculateFrequencyRatio(PitchDriftInSemitones));

        public float volume = 0.7f;

        public void Play(ESoundClip eSoundClip, AudioClip overrideClip = null, float volumeScale = 1f, float pitch = 1)
        {
            if (eSoundClip == ESoundClip.None) return;
            var audioClip = overrideClip;
            if (audioClip == null) audioClip = Resources.Load<AudioClip>($"GazeIXSounds/{eSoundClip.ToString()}");

            if (audioClip)
            {
                _audioSource.pitch = pitch;
                _audioSource.PlayOneShot(audioClip, volume * volumeScale);
            }
        }


        public static float CalculateFrequencyRatio(int semitones)
        {
            const float twelfthRootOfTwo = 1.0594630943592953f; // 12th root of 2
            return Mathf.Pow(twelfthRootOfTwo, semitones);
        }
    }
}
