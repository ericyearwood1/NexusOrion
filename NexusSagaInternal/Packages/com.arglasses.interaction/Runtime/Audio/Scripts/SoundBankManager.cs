using System;
using System.Collections.Generic;
using UnityEngine;

/*
Prototyping tools to enable faster audio iteration on ARGlasses

"author": {
    "name": "Dan Rosas",
    "email": "danrosas@fb.com",
    "url": "https://www.internalfb.com/profile/view/571628801"
},
 */

namespace ARGlasses.Interaction
{
    public class SoundBankManager : MonoBehaviour
    {
        public string ActiveBank;
        public AudioEventCollection[] SoundBanks;
        public AudioSource Source;
        public GameObject SpatialAudioOneShotPrefab;

        [Serializable]
        public struct AudioEventCollection
        {
            public string BankName;
            public AudioEvent[] AudioEvents;
        }

        [Serializable]
        public struct AudioEvent
        {
            public string Name;
            public AudioClip[] Clips;
        }

        private Dictionary<string, Dictionary<string, AudioEvent>> _audioDictionary = new();

        void Awake()
        {
            GenerateDictionary();
            CheckDefaultBank();
        }

        private bool _started;
        void Start()
        {
            _started = true;
        }

        public void Play(string eventName)
        {
            Play(eventName, 0f);
        }

        public void Play(string eventName, float pan = 0f)
        {
            AudioClip clip = GetClip(eventName);
            PlayClip(clip, pan);
        }

        public void PlayClip(string eventName, int clipIndex, float pan = 0f)
        {
            AudioClip clip = GetClip(eventName, clipIndex);
            PlayClip(clip, pan);
        }

        private void PlayClip(AudioClip clip, float pan)
        {
            if (!_started || clip == null) return;
            Source.panStereo = pan;
            Source.clip = clip;
            Source.Play();
        }

        public AudioClip GetClip(string clipName, int clipIndex = -1)
        {
            Dictionary<string, AudioEvent> events;
            if (!_audioDictionary.TryGetValue(ActiveBank, out events)) return null;

            AudioEvent ae;
            if (events.TryGetValue(clipName, out ae)) return clipIndex < 0 ? ae.Clips[UnityEngine.Random.Range(0, ae.Clips.Length)] : ae.Clips[clipIndex];

            Debug.Log("SoundBankManager.Play: Tried to access event that does not exist in the Active Bank: " + clipName);
            return null;
        }

        public void NextBank()
        {
            if (SoundBanks.Length <= 0)
            {
                Debug.Log("SoundBankManager.NextBank: Error - No sound banks defined.");
                return;
            }

            int currentBank = 0;
            for (int i = 0; i < SoundBanks.Length; ++i)
            {
                if (SoundBanks[i].BankName != ActiveBank) continue;
                if (i == SoundBanks.Length - 1) currentBank = 0;
                else currentBank = i + 1;
                break;
            }

            ActiveBank = SoundBanks[currentBank].BankName;
        }

        public void PrevBank()
        {
            if (SoundBanks.Length <= 0)
            {
                Debug.Log("SoundBankManager.PrevBank: Error - No sound banks defined.");
                return;
            }

            int currentBank = 0;
            for (int i = 0; i < SoundBanks.Length; ++i)
            {
                if (SoundBanks[i].BankName != ActiveBank) continue;

                if (i == 0) currentBank = SoundBanks.Length - 1;
                else currentBank = i - 1;
                break;
            }

            ActiveBank = SoundBanks[currentBank].BankName;
        }

        private void GenerateDictionary()
        {
            foreach (AudioEventCollection aec in SoundBanks)
            {
                Dictionary<string, AudioEvent> d = new Dictionary<string, AudioEvent>();
                foreach (AudioEvent ae in aec.AudioEvents) d.Add(ae.Name, ae);
                _audioDictionary.Add(aec.BankName, d);
            }
        }

        private void CheckDefaultBank()
        {
            if (ActiveBank == String.Empty && SoundBanks.Length > 0) ActiveBank = SoundBanks[0].BankName;
        }

        public void PlayAtLocation(string eventName, Vector3 pos)
        {
            if (SpatialAudioOneShotPrefab == null)
            {
                Debug.Log("SpatialAudioOneShot prefab is missing from this instance of SoundBankManager. Please add it from the [Orion] Audio/Prefabs folder.");
                return;
            }
            AudioClip clip = GetClip(eventName);
            GameObject go = GameObject.Instantiate(SpatialAudioOneShotPrefab);
            go.GetComponent<AudioSource>().clip = clip;
            go.transform.position = pos;
        }
    }
}
