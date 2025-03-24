using System;
using TMPro;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.UI;
using UnityEngine.Video;

namespace ARGlasses.Interaction
{
    [Serializable]
    [UnityEngine.Scripting.Preserve]
    public class VideoModel : MonoBehaviour
    {
        [SerializeField] private TMP_Text _timeStampText;
        [SerializeField] private Color _pauseColor = Color.gray;
        [SerializeField] private Color _tint = Color.white;
        [SerializeField, ReadOnly] private bool _isPaused;

        private RenderTexture lastTexture = null;
        public Color Tint
        {
            get => _tint;
            set => _tint = value;
        }

        [SerializeField, ReadOnly] private RawImage _rawImage;

        public RawImage RawImage
        {
            get => _rawImage;
            set => _rawImage = value;
        }

        [SerializeField, ReadOnly] private VideoPlayer _videoPlayer;
        public VideoPlayer VideoPlayer => _videoPlayer;

        public float Volume
        {
            get => _videoPlayer.GetDirectAudioVolume(0);
            set
            {
                //this triggers endless loop
                //Debug.Log("videomodel volume func");
                if (value == Volume) return;
                _videoPlayer.SetDirectAudioVolume(0, value);
                WhenVolumeChanged(Volume);
                //Debug.Log("videomodel volume func complete");
            }
        }

        public bool isMuted
        {
            get
            {
                ushort trackIndex = 0; // Assuming target the first audio track
                return _videoPlayer.GetDirectAudioMute(trackIndex);
            }
            set
            {
                ushort trackIndex = 0; 
                _videoPlayer.SetDirectAudioMute(trackIndex, value);
            }
        }

        public event Action<float> WhenVolumeChanged = delegate { };

        public bool IsPaused
        {
            get => _isPaused;
            set
            {
                _isPaused = value;
                IsPlaying = !value;
            }
        }

        public bool IsPlaying
        {
            get => _videoPlayer.isPlaying;
            set
            {
                if (_videoPlayer.isPlaying == value) { return; }
                //Debug.LogWarning("playing: " + value + " : " + _videoPlayer.clip.name);
                Assert.IsNotNull(_rawImage);
                Assert.IsNotNull(_videoPlayer.clip);
                if (value) _videoPlayer.Play();
                else _videoPlayer.Pause();

                WhenPlayStateChanged(value);
            }
        }

        public event Action<bool> WhenPlayStateChanged = delegate { };

        public VideoClip VideoClip
        {
            get => _videoPlayer.clip;
            set
            {
                if(_videoPlayer.clip == value){return;}

                _videoPlayer.clip = value;
                // var renderTexture = new RenderTexture((int)_videoPlayer.width, (int)_videoPlayer.height, 24, GraphicsFormat.R8G8B8A8_UNorm, 0);
                var renderTexture = new RenderTexture((int)_videoPlayer.width, (int)_videoPlayer.height, 24);
                
                _videoPlayer.targetTexture = renderTexture;
                RawImage.texture = renderTexture;
                if(lastTexture != null){
                    lastTexture.Release();
                    Destroy(lastTexture);
                }
                lastTexture = renderTexture;
                RawImage.maskable = true;
                WhenVideoClipChanged(value);
            }
        }

        public event Action<VideoClip> WhenVideoClipChanged = delegate { };

        protected void Awake()
        {
            _videoPlayer = this.Ensure<VideoPlayer>();
            _videoPlayer.source = VideoSource.VideoClip;
            _videoPlayer.isLooping = true;
            _videoPlayer.playOnAwake = false;
            _videoPlayer.Pause();
            this.Descendant(ref _timeStampText, optional: true);
        }
        void OnDestroy(){
            if(lastTexture != null){
                lastTexture.Release();
                Destroy(lastTexture);
            }
            
        }

        protected void Start()
        {
            this.Descendant(ref _rawImage);
        }

        void Update()
        {
            // RawImage.color = Color.Lerp(RawImage.color, _isPaused ?  _pauseColor : _tint, Time.time * 0.5f);
            // if (_timeStampText) _timeStampText.text = $"{(int)_videoPlayer.time / 60:00}:{_videoPlayer.time % 60:00}s";
        }
    }
}
