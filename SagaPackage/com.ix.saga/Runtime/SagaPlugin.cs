// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;
using UnityEngine.Events;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace IX.Saga
{

    public enum FrameDataType : byte
    {
        /// Undefined data type.
        DT_UNDEFINED = 0,
        /// Unsigned 8 bit integer data type (uint8_t).
        DT_UNSIGNED_INTEGER_8 = 1,
        /// Signed 8 bit integer data type (int8_t).
        DT_SIGNED_INTEGER_8 = 2,
        /// Unsigned 16 bit integer data type (uint16_t).
        DT_UNSIGNED_INTEGER_16 = 3,
        /// Signed 16 bit integer data type (int16_t).
        DT_SIGNED_INTEGER_16 = 4,
        /// Unsigned 32 bit integer data type (uint32_t).
        DT_UNSIGNED_INTEGER_32 = 5,
        /// Signed 232 bit integer data type (int32_t).
        DT_SIGNED_INTEGER_32 = 6,
        /// Unsigned 64 bit integer data type (uint64_t).
        DT_UNSIGNED_INTEGER_64 = 7,
        /// Signed 64 bit integer data type (int64_t).
        DT_SIGNED_INTEGER_64 = 8,
        /// Signed 16 bit float data type.
        DT_SIGNED_FLOAT_16 = 9,
        /// Signed 32 bit float data type (float).
        DT_SIGNED_FLOAT_32 = 10,
        /// Signed 64 bit float data type (double).
        DT_SIGNED_FLOAT_64 = 11,
        /// The helper data type which can be used to identify the last defined data type, DT_END is exclusive.
        DT_END = 12
    }

    [System.Serializable]
    public class SagaPluginConfiguration {
        public bool debug = true;
        public bool streamingColor = false;
        public bool streamingAudio = false;
        public bool streamingDepth = false;
        public bool streamingSpeech2Text = false;
        public bool supportText2Speech = false;
        public bool supportImage = false;
        public float targetFrequencyFrame = 30.0f;
        public bool generateDepthmap = false;
        [Range (1, 2)]
        public int maxEyeCount = 1;
        public string pixieServerIp = "192.168.0.99";
    };

    public struct FrameData
    {
        public int index;
        public Texture2D texture;
        public IntPtr data;
        public int dataSize;
        public int width;
        public int height;
        public int channels;
        public byte dataType;
        public int bytesPerDataType;
        public TextureFormat textureFormat;
    }

    public class SagaPlugin : MonoBehaviour
    {
        public SagaPluginConfiguration _configuration = new SagaPluginConfiguration();
        private SagaSettings _settings = new SagaSettings();

        private bool _isValid = false;

        public bool isValid { get => _isValid; }

        private IntPtr pluginHandler_ = IntPtr.Zero;
        public IntPtr PluginHandler { get => pluginHandler_; }

        private readonly Dictionary<int, FrameData> _frames = new();

        private static class OVRP_Native
        {
            private const string pluginName = "OVRPlugin";

            [DllImport(pluginName, CallingConvention = CallingConvention.Cdecl)]
            public static extern OVRPlugin.Result ovrp_GetNativeSDKPointer2(out IntPtr nativeSDKPointer);

            [DllImport(pluginName, CallingConvention = CallingConvention.Cdecl)]
            public static extern OVRPlugin.Result ovrp_GetNativeOpenXRHandles(out UInt64 xrInstance, out UInt64 xrSession);
        }


        public static void HexagonEnvInject() {
#if PLATFORM_ANDROID && !UNITY_EDITOR
            // Add path to our apk's libraries to use our packaged hexagon instead of the system installed one to prevent ABI mismatches.
            if (isAndroid12()) {
                Environment.SetEnvironmentVariable("ADSP_LIBRARY_PATH",
                    $"/vendor/lib/rfsa/adsp/;/system/lib/rfsa/adsp/");
                Environment.SetEnvironmentVariable("BOLT_HEXAGON_LIBRARY_PATH", "/vendor/lib64");
            } else {
                string libraryPath = Path.Combine(Path.GetDirectoryName(Application.dataPath) ?? string.Empty, "lib/arm64");
                Environment.SetEnvironmentVariable("ADSP_LIBRARY_PATH",
                    $"{libraryPath};/vendor/lib/rfsa/adsp/;/system/lib/rfsa/adsp/");
                Environment.SetEnvironmentVariable("BOLT_HEXAGON_LIBRARY_PATH", libraryPath);
                string ldPath = Environment.GetEnvironmentVariable("LD_LIBRARY_PATH");
                if (ldPath == null)
                {
                    ldPath = libraryPath;
                }
                else
                {
                    ldPath = $"{libraryPath}:{ldPath}";
                }
                Environment.SetEnvironmentVariable("LD_LIBRARY_PATH", ldPath);
            }
#endif
        }

        private static bool isAndroid12() {
            var operatingSystem = SystemInfo.operatingSystem.Split(" ");
            var androidVersionNumber = operatingSystem.Length >= 3 ? operatingSystem[2] : "";
            return androidVersionNumber == "12";
        }

        private static IntPtr GetNativePointer()
        {
            IntPtr nativePointer = IntPtr.Zero;
            OVRPlugin.Result ret = OVRP_Native.ovrp_GetNativeSDKPointer2(out nativePointer);
            Debug.Log($"IX: SAGA: IX.Saga.SagaPlugin.GetNativePointer ovrp_GetNativeSDKPointer2 was called and returned[{ret}]");
            /*if (nativePointer == IntPtr.Zero)
            {
                throw new Exception("Initialization Failed: " +
                "Failed to obtain native SDK pointer.");
            }
            */
            return nativePointer;
        }


        // Start is called before the first frame update
        void Start()
        {
            Debug.Log("IX: SAGA: IX.Saga.SagaPlugin.Start BEGIN");
            Debug.Log("IX: SAGA: IX.Saga.SagaPlugin.Start END");
        }

        private void Awake()
        {
            // HexagonEnvInject();

            Debug.Log("IX: SAGA: IX.Saga.SagaPlugin.Awake BEGIN");
            IntPtr nativePointer = GetNativePointer();

            if (nativePointer == IntPtr.Zero)
            {
#if UNITY_EDITOR
                Debug.LogError("IX: SAGA: IX.Saga.SagaPlugin.Awake Initialization has failed but will continue for debug");
#else
                Debug.LogError("IX: SAGA: IX.Saga.SagaPlugin.Awake Initialization Failed: Failed to obtain native SDK pointer");
                //return;
#endif
            }

            OVRP_Native.ovrp_GetNativeOpenXRHandles(out ulong xrInstance, out ulong xrSession);
            if (xrInstance == 0)
            {
#if UNITY_EDITOR
                Debug.LogError("IX: SAGA: IX.Saga.SagaPlugin.Awake Initialization2 has failed but will continue for debug");
#else
                Debug.LogError("IX: SAGA: IX.Saga.SagaPlugin.Awake Initialization2 Failed: Failed to obtain native SDK pointer");
                //return;
#endif
            }

            _settings.debug = _configuration.debug;
            _settings.streamingColor = _configuration.streamingColor;
            _settings.streamingAudio = _configuration.streamingAudio;
            _settings.streamingDepth = _configuration.streamingDepth;
            _settings.streamingSpeech2Text = _configuration.streamingSpeech2Text;
            _settings.supportText2Speech = _configuration.supportText2Speech;
            _settings.supportImage = _configuration.supportImage;
            _settings.targetFrequencyFrame = _configuration.targetFrequencyFrame;
            _settings.pixieServerIp = new string(_configuration.pixieServerIp);
            _settings.generateDepthmap = _configuration.generateDepthmap;
            _settings.maxEyeCount = _configuration.maxEyeCount;

            Debug.Log($"IX: SAGA: IX.Saga.SagaPlugin.Awake _settings.pixieServerIp = {_settings.pixieServerIp}");

            pluginHandler_ = Native.InitPlugin(nativePointer, _settings);
            if (pluginHandler_ == IntPtr.Zero)
            {
                Debug.LogError("IX: SAGA: IX.Saga.SagaPlugin.Awake Failed to Initialize Plugin");
                return;
            }
            else
            {
                Debug.Log("IX: SAGA: IX.Saga.SagaPlugin.Awake Initialized Plugin");
            }
            //Debug.Log($"{nameof(FrameProvider)} - CameraStart with camera index {cameraIndex}");
            //if (!Native.CameraStart(pluginHandler_, cameraIndex))
            //    Debug.LogError("CameraStart failed to start!");
            Debug.Log("IX: SAGA: IX.Saga.SagaPlugin.Awake END");
        }

        void OnDestroy()
        {
            Debug.Log("IX: SAGA: IX.Saga.SagaPlugin.OnDestroy BEGIN");
            //_unityLog?.Dispose();
            if (pluginHandler_ != IntPtr.Zero)
            {
                Native.DisposePlugin(pluginHandler_);
                pluginHandler_ = IntPtr.Zero;
            }
            Debug.Log("IX: SAGA: IX.Saga.SagaPlugin.OnDestroy END");
        }

        private void OnEnable()
        {

        }

        // Update is called once per frame
        void Update()
        {
            Debug.Log("IX: SAGA: IX.Saga.SagaPlugin.Update BEGIN");
            Native.UpdatePlugin(pluginHandler_);
            Debug.Log("IX: SAGA: IX.Saga.SagaPlugin.Update END");
        }

        private TextureFormat GetTextureFormat(int channels, byte imageDataType,  int imageBytesPerDataType)
        {
            FrameDataType frameDataType = (FrameDataType)imageDataType;
            TextureFormat textureFormat = TextureFormat.RGB24;
            if (channels == 1)
            {
                if ((frameDataType == FrameDataType.DT_UNSIGNED_INTEGER_8)||
                    (frameDataType == FrameDataType.DT_SIGNED_INTEGER_8))
                {
                    textureFormat = TextureFormat.R8;
                }
                else if ((frameDataType == FrameDataType.DT_UNSIGNED_INTEGER_16)||
                    (frameDataType == FrameDataType.DT_SIGNED_INTEGER_16))
                {
                    textureFormat = TextureFormat.R16;
                }
                else if (frameDataType == FrameDataType.DT_SIGNED_FLOAT_16)
                {
                    textureFormat = TextureFormat.RHalf;
                }
                else if (frameDataType == FrameDataType.DT_SIGNED_FLOAT_32)
                {
                    textureFormat = TextureFormat.RFloat;
                }
            }
            else if (channels == 2)
            {
                if ((frameDataType == FrameDataType.DT_UNSIGNED_INTEGER_8)||
                    (frameDataType == FrameDataType.DT_SIGNED_INTEGER_8))
                {
                    textureFormat = TextureFormat.RG16;
                }
                else if ((frameDataType == FrameDataType.DT_UNSIGNED_INTEGER_16)||
                    (frameDataType == FrameDataType.DT_SIGNED_INTEGER_16))
                {
                    textureFormat = TextureFormat.RG32;
                }
                else if (frameDataType == FrameDataType.DT_SIGNED_FLOAT_16)
                {
                    textureFormat = TextureFormat.RGHalf;
                }
                else if (frameDataType == FrameDataType.DT_SIGNED_FLOAT_32)
                {
                    textureFormat = TextureFormat.RGFloat;
                }
            }
            else if (channels == 3)
            {
                if ((frameDataType == FrameDataType.DT_UNSIGNED_INTEGER_8)||
                    (frameDataType == FrameDataType.DT_SIGNED_INTEGER_8))
                {
                    textureFormat = TextureFormat.RGB24;
                }
                else if ((frameDataType == FrameDataType.DT_UNSIGNED_INTEGER_16)||
                    (frameDataType == FrameDataType.DT_SIGNED_INTEGER_16))
                {
                    textureFormat = TextureFormat.RGB48;
                }
            }
            else if (channels == 4)
            {
                if ((frameDataType == FrameDataType.DT_UNSIGNED_INTEGER_8)||
                    (frameDataType == FrameDataType.DT_SIGNED_INTEGER_8))
                {
                    textureFormat = TextureFormat.RGBA32;
                }
                else if ((frameDataType == FrameDataType.DT_UNSIGNED_INTEGER_16)||
                    (frameDataType == FrameDataType.DT_SIGNED_INTEGER_16))
                {
                    textureFormat = TextureFormat.RGBA64;
                }
                else if (frameDataType == FrameDataType.DT_SIGNED_FLOAT_16)
                {
                    textureFormat = TextureFormat.RGBAHalf;
                }
                else if (frameDataType == FrameDataType.DT_SIGNED_FLOAT_32)
                {
                    textureFormat = TextureFormat.RGBAFloat;
                }
            }
            return textureFormat;
        }

        private FrameData CreateFrame(int frameIndex, int width, int height, int channels,
                                    byte imageDataType,  int imageBytesPerDataType, int imageDataSize)
        {
            Debug.Log($"IX: SAGA: IX.Saga.SagaPlugin.InitFrame[{frameIndex}] create texture {width}x{height}x{channels}x{imageBytesPerDataType}");
            var frame = new FrameData();
            frame.index = frameIndex;
            frame.width = width;
            frame.height = height;
            frame.channels = channels;
            frame.dataType = imageDataType;
            frame.bytesPerDataType = imageBytesPerDataType;
            frame.textureFormat = GetTextureFormat(channels, imageDataType, imageBytesPerDataType);
            frame.texture = new Texture2D(width, height, frame.textureFormat, false);
            frame.dataSize = Math.Max(width * height * channels * imageBytesPerDataType, imageDataSize);
            frame.data = Marshal.AllocHGlobal(frame.dataSize);
            if (_frames.TryGetValue(frameIndex, out FrameData frameOld)){
                // frameOld.texture.Release();
                Marshal.FreeHGlobal(frameOld.data);
                _frames.Remove(frameIndex);
            }
            _frames.Add(frameIndex, frame);
            return frame;
        }

        public bool PollFrame(int frameIndex, out Texture2D outputTexture)
        {
            outputTexture = null;
            if (Native.GetDebugFrameInfo(
                pluginHandler_,
                frameIndex,
                out int imageWidth,
                out int imageHeight,
                out int imageChannels,
                out byte imageDataType,
                out int imageBytesPerDataType,
                out int imageDataSize))
            {
                bool ret = false;
                if (!_frames.TryGetValue(frameIndex, out FrameData frame) ||
                    frame.width != imageWidth ||
                    frame.height != imageHeight ||
                    frame.channels != imageChannels ||
                    frame.dataType != imageDataType ||
                    frame.bytesPerDataType != imageBytesPerDataType ||
                    frame.dataSize != imageDataSize)
                {
                    Debug.Log($"IX: SAGA: IX.Saga.SagaPlugin.PollFrame[{frameIndex}] new frame definition {imageWidth}x{imageHeight}x{imageChannels}x{imageBytesPerDataType}");
                    frame = CreateFrame(frameIndex, imageWidth, imageHeight, imageChannels, imageDataType, imageBytesPerDataType,  imageDataSize);
                    outputTexture = frame.texture;
                    ret = true;
                }
                if (Native.GetDebugFrameData(pluginHandler_, frameIndex, frame.data, imageDataSize, out int copiedByte))
                {
                    Debug.Log($"IX: SAGA: IX.Saga.SagaPlugin.PollFrame[{frameIndex}] data {copiedByte} bytes copied");
                    frame.texture.LoadRawTextureData(frame.data, copiedByte);
                    frame.texture.Apply();
                    ret = true;
                }
                return ret;
            }
            Debug.LogError($"IX: SAGA: IX.Saga.SagaPlugin.PollFrame[{frameIndex}] no data");
            return false;
        }
    }
}
