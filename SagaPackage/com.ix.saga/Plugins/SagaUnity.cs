// Oculus VR, LLC Proprietary and Confidential.

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;


namespace IX.Saga
{
    /// native C++ plugin
    public static class Native
    {
        const string DLL = "SagaPlugin";

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr InitPlugin(IntPtr nativeSDKPointer, in SagaSettings videoSettings);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DisposePlugin(IntPtr pluginHandle);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetRenderEventFunc();

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern void UpdatePlugin(IntPtr pluginHandle);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetStringSetting(IntPtr pluginHandle, Byte item, [MarshalAs(UnmanagedType.LPStr)] string value);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetBoolSetting(IntPtr pluginHandle, Byte item, bool value);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetIntSetting(IntPtr pluginHandle, Byte item, int value);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetFloatSetting(IntPtr pluginHandle, Byte item, float value);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetDebugFrameInfo(
        IntPtr pluginHandle,
        int index,
        out int imageWidth,
        out int imageHeight,
        out int imageChannel,
        out byte imageDataType,
        out int imageBytesPerDataType,
        out int imageDataSize);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetDebugFrameData(IntPtr pluginHandle,
        // int index, [MarshalAs(UnmanagedType.LPArray)] byte[] imageData, int dataSize, out int copiedByte);
        int index, IntPtr imageData, int dataSize, out int copiedByte);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetDebugFrameCameraInfo(
        IntPtr sagaHandle,
        int index,
        out int cameraType,
        out short imageWidth,
        out short imageHeight,
        IntPtr camParams,
        int camParamsSize,
        out int copiedParams);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetDebugFramePoseTCameraWorld(
        IntPtr sagaHandle,
        int index,
        IntPtr pose,
        int poseParamsSize,
        out int copiedParams);

        [DllImport(DLL, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetDebugFramePoseCameraFromDevice(
        IntPtr sagaHandle,
        int index,
        IntPtr pose,
        int poseParamsSize,
        out int copiedParams);
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SagaSettings {
        [MarshalAs(UnmanagedType.U1)]
        public bool debug;
        [MarshalAs(UnmanagedType.U1)]
        public bool streamingColor;
        [MarshalAs(UnmanagedType.U1)]
        public bool streamingAudio;
        [MarshalAs(UnmanagedType.U1)]
        public bool streamingDepth;
        [MarshalAs(UnmanagedType.U1)]
        public bool streamingSpeech2Text;
        [MarshalAs(UnmanagedType.U1)]
        public bool supportText2Speech;
        [MarshalAs(UnmanagedType.U1)]
        public bool supportImage;
        public float targetFrequencyFrame;
        [MarshalAs(UnmanagedType.U1)]
        public bool generateDepthmap;
        public int maxEyeCount;
        [MarshalAs(UnmanagedType.LPStr)]
        public string pixieServerIp;
    };

    public enum SagaSettingItem : byte
    {
        SI_UNDEFINED = 0,
        SI_SERVER_IP = 1,
        SI_COLOR_IP = 2,
        SI_COLOR_PORT = 3,
        SI_COLOR_ENABLED = 4,
        SI_AUDIO_IP = 5,
        SI_AUDIO_PORT = 6,
        SI_AUDIO_ENABLED = 7,
        SI_TEXT2SPEECH_IP = 8,
        SI_TEXT2SPEECH_PORT = 9,
        SI_TEXT2SPEECH_ENABLED = 10,
        SI_SPEECH2TEXT_IP = 11,
        SI_SPEECH2TEXT_PORT = 12,
        SI_SPEECH2TEXT_ENABLED = 13,
        SI_IMAGE_IP = 14,
        SI_IMAGE_PORT = 15,
        SI_IMAGE_ENABLED = 16,
        SI_DEPTH_IP = 17,
        SI_DEPTH_PORT = 18,
        SI_DEPTH_ENABLED = 19,
        SI_PROCESS_FPS = 20, // float
        SI_GENERATE_DEPTHMAP = 21, // bool
        SI_MAX_EYE_COUNT = 22, // int
        SI_COUNT = 23
    }
}
