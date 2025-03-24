using System;
using ARGlasses.Interaction;
using ARGlasses.Interaction.Motion;
using UnityEngine;

namespace ARGlasses.Components
{
    [Serializable]
    public class TooltipViewModel : ViewModelBase
    {
        [SerializeField] private Target _sourceTarget; // The interaction model of the component we're pointing at
        
        [Header("Content")]
        [SerializeField] private Sprite _icon;
        [SerializeField] private string _titleText;
        [SerializeField] private string _descriptionText;
        
        [Header("Timing")]
        [SerializeField] private float _spawnDelay = 0.75f;
        [SerializeField] private float _despawnDelay = 0.75f;

        [Space(10)]
        [SerializeField] private MotionParamsEasing _easeIn = new(Easing.OutCubic, 0.5f);
        //[SerializeField] private MotionParams _easeIn = new MotionParams(1f, 0.9f, 1f);

        [Space(10)]
        [SerializeField] private MotionParamsEasing _easeOut = new(Easing.InCubic, 0.75f);
        //[SerializeField] private MotionParams _easeOut = new MotionParams(1f, 0.9f, 0f);
        
        [Header("Transform Properties")]
        [SerializeField] private FromDirection _fromDirection;
        [Tooltip("Offset in pixels.")]
        [SerializeField] private float _offset = 20f;
        [SerializeField] private float _hiddenTranslation = 0.5f;
        [SerializeField] private float _hiddenScale = 0.5f;
        
        public Target SourceTarget
        {
            get => _sourceTarget;
            set => SetField(ref _sourceTarget, value);
        }
        
        public string TitleText
        {
            get => _titleText;
            set => SetField(ref _titleText, value);
        }

        public string DescriptionText
        {
            get => _descriptionText;
            set => SetField(ref _descriptionText, value);
        }

        public Sprite IconSprite
        {
            get => _icon;
            set => SetField(ref _icon, value);
        }
        
        public float SpawnDelay
        {
            get => _spawnDelay;
            set => SetField(ref _spawnDelay, value);
        }
        
        public float DespawnDelay
        {
            get => _despawnDelay;
            set => SetField(ref _despawnDelay, value);
        }

        public MotionParamsEasing EaseIn
        {
            get => _easeIn;
            set => SetField(ref _easeIn, value);
        }
        
        public MotionParamsEasing EaseOut
        {
            get => _easeOut;
            set => SetField(ref _easeOut, value);
        }
        
        // public float FadeInDuration
        // {
        //     get => _easeIn._duration;
        //     set => SetField(ref _easeIn._duration, value);
        // }
        //
        // public float FadeOutDuration
        // {
        //     get => _easeOut._duration;
        //     set => SetField(ref _easeOut._duration, value);
        // }
        
        public FromDirection FromDirection
        {
            get => _fromDirection;
            set => SetField(ref _fromDirection, value);
        }
        
        public float Offset
        {
            get => _offset;
            set => SetField(ref _offset, value);
        }

        public float HiddenScale
        {
            get => _hiddenScale;
            set => SetField(ref _hiddenScale, value);
        }
        
        public float HiddenTranslation
        {
            get => _hiddenTranslation;
            set => SetField(ref _hiddenTranslation, value);
        }
    }
}