/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if OVRPLUGIN_TESTING

using System.Collections;
using System.Runtime.InteropServices;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System.Linq;
using System.Text;
using System.Reflection;

internal class OVRPassthroughColorLutTests : OVRPluginEditModeTest
{
#region Test data

    private Color[] _lutTextureInput = new Color[]
    {
        Color.black, Color.black, Color.black, Color.black,
        Color.black, Color.black, Color.blue, Color.blue
    };

    private Color[] _flippedLutInput = new Color[]
    {
        Color.black, Color.black, Color.blue, Color.blue,
        Color.black, Color.black, Color.black, Color.black
    };

    private Color[] _lutColorArrayInput = new Color[]
    {
        Color.black, Color.black, Color.black, Color.black,
        Color.blue, Color.blue, Color.black, Color.black
    };

    private Color[] _altLutColorArrayInput = new Color[]
    {
        Color.black, Color.black, Color.black, Color.black,
        Color.red, Color.red, Color.black, Color.black
    };

    private Color32[] _lutColor32ArrayInput = new Color32[]
    {
        Color.black, Color.black, Color.black, Color.black,
        Color.blue, Color.blue, Color.black, Color.black
    };

    /// <summary>
    /// 8 by 8 square input array
    /// </summary>
    private byte[] _squareLutByteArrayInput = new byte[]
    {
        0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
    };

    /// <summary>
    /// Expected output for 8 by 8 square input array
    /// </summary>
    private byte[] _expectedSquareLutOutput = new byte[]
    {
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 255, 0,
        0, 255, 0, 0, 255, 0, 0, 255
    };

    private byte[] _expectedLutOutput = new byte[]
    {
        0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 255,
        0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 0, 255, 0, 0, 0, 255
    };

    private byte[] _altExpectedLutOutput = new byte[]
    {
        0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 255,
        255, 0, 0, 255, 255, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 255
    };

    private byte[] _expectedRGBLutOutput = new byte[]
    {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 255, 0, 0, 255, 0, 0, 0, 0, 0, 0
    };

#endregion

    private FakeOVRPlugin84 _fakeOVRPlugin84;
    private FakeOVRPlugin85 _fakeOVRPlugin85;
    private FakeOVRPlugin66 _fakeOVRPlugin66;
    private MyLogHandler _myLogHandler;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        _fakeOVRPlugin84 = new FakeOVRPlugin84();
        OVRPlugin.OVRP_1_84_0.mockObj = _fakeOVRPlugin84;

        _fakeOVRPlugin85 = new FakeOVRPlugin85();
        OVRPlugin.OVRP_1_85_0.mockObj = _fakeOVRPlugin85;

        _fakeOVRPlugin66 = new FakeOVRPlugin66();
        OVRPlugin.OVRP_1_66_0.mockObj = _fakeOVRPlugin66;

        _myLogHandler = new MyLogHandler();
        Debug.unityLogger.logHandler = _myLogHandler;
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        _myLogHandler.AssertHandled();
        Debug.unityLogger.logHandler = _myLogHandler.DefaultLogHandler;
        OVRPlugin.OVRP_1_84_0.mockObj = new OVRPlugin.OVRP_1_84_0_TEST();
        OVRPlugin.OVRP_1_85_0.mockObj = new OVRPlugin.OVRP_1_85_0_TEST();
        OVRPlugin.OVRP_1_66_0.mockObj = new OVRPlugin.OVRP_1_66_0_TEST();
        yield return base.UnityTearDown();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestCreateWithTexture()
    {
        var colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(16));
        Assert.IsTrue(colorLut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestCreateWithLargeResolutionTextureFails()
    {
        OVRPassthroughColorLut colorLut = null;
        Assert.Throws<System.ArgumentException>(() =>
        {
            colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(65));
        });
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestCreateWithNonPowerOf2ResolutionTextureFails()
    {
        OVRPassthroughColorLut colorLut = null;
        Assert.Throws<System.ArgumentException>(() =>
        {
            colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(5));
        });
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestCreateWithColorArray()
    {
        var texture = GetBlankTextureOfResolution(16);
        var colorLut = new OVRPassthroughColorLut(texture.GetPixels(), OVRPassthroughColorLut.ColorChannels.Rgba);
        Assert.IsTrue(colorLut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestCreateWithColor32Array()
    {
        var texture = GetBlankTextureOfResolution(16);
        var colorLut = new OVRPassthroughColorLut(texture.GetPixels32(), OVRPassthroughColorLut.ColorChannels.Rgba);
        Assert.IsTrue(colorLut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestCreateWithByteArray()
    {
        var texture = GetBlankTextureOfResolution(16);
        var data = texture.GetPixelData<byte>(0).ToArray();
        var colorLut = new OVRPassthroughColorLut(data, OVRPassthroughColorLut.ColorChannels.Rgba);
        Assert.IsTrue(colorLut.IsValid);
        yield return null;
    }

    /// <summary>
    /// Raw pixel data is accessed when creating Color LUTs from texture.
    /// Creation of Color LUTs depends on linear and gamma color space textures
    /// returning the same color information, this test makes sure that Unity has not
    /// changed that.
    /// </summary>
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestLinearTextureDataEqualToGamma()
    {
        var resolution = 16;
        var size = resolution * resolution * resolution;
        var colors = new Color[size];
        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = Color.white * ((float)i / size);
        }

        var texture = new Texture2D(resolution * resolution, resolution, TextureFormat.RGBA32, false, false);
        texture.SetPixels(colors);
        texture.Apply();
        var data = texture.GetPixelData<byte>(0).ToArray();

        var texture2 = new Texture2D(resolution * resolution, resolution, TextureFormat.RGBA32, false, true);
        texture2.SetPixels(colors);
        texture2.Apply();
        var data2 = texture2.GetPixelData<byte>(0).ToArray();

        for (int i = 0; i < data.Length; i++)
        {
            Assert.AreEqual(data[i], data2[i]);
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestTextureChannelInterpretedCorrectly()
    {
        var resolution = 16;
        var texture = new Texture2D(resolution * resolution, resolution, TextureFormat.RGBA32, false, false);
        var lut = new OVRPassthroughColorLut(texture);
        Assert.AreEqual(lut.Channels, OVRPassthroughColorLut.ColorChannels.Rgba);

        var texture2 = new Texture2D(resolution * resolution, resolution, TextureFormat.RGB24, false, true);
        lut = new OVRPassthroughColorLut(texture2);
        Assert.AreEqual(lut.Channels, OVRPassthroughColorLut.ColorChannels.Rgb);

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestFailsCreatingWithUnsuportedTextureFormat()
    {
        var resolution = 16;
        var texture = new Texture2D(resolution * resolution, resolution, TextureFormat.ARGB4444, false, false);

        Assert.Throws<System.ArgumentException>(() => new OVRPassthroughColorLut(texture));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestBufferAssigned()
    {
        InitializeInsightPassthrough();
        var resolution = 2;
        var texture = new Texture2D(resolution * resolution, resolution);
        OVRPlugin.PassthroughColorLutData _data = default;
        _fakeOVRPlugin84.CreateCallback = (channels, resolution, data) =>
        {
            _data = data;
            return OVRPlugin.Result.Success;
        };

        var lut = new OVRPassthroughColorLut(texture);
        var expectedSize = resolution * resolution * resolution * 4;

        Assert.AreEqual(_data.BufferSize, expectedSize);
        byte[] managedArray = new byte[expectedSize];
        Assert.DoesNotThrow(() => Marshal.Copy(_data.Buffer, managedArray, 0, expectedSize));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestExpectedUnflippedTextureOutput()
    {
        var resolution = 2;
        var texture = new Texture2D(resolution * resolution, resolution);
        texture.SetPixels(_lutTextureInput);
        texture.Apply();

        AssertExpectedDataOnCreate(_lutTextureInput.Length * 4, _expectedLutOutput);
        var lut = new OVRPassthroughColorLut(texture);
        Assert.IsTrue(lut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestExpectedFlippedTextureOutput()
    {
        var resolution = 2;
        var texture = new Texture2D(resolution * resolution, resolution);
        texture.SetPixels(_flippedLutInput);
        texture.Apply();

        AssertExpectedDataOnCreate(_flippedLutInput.Length * 4, _expectedLutOutput);
        var lut = new OVRPassthroughColorLut(texture, false);
        Assert.IsTrue(lut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestExpectedColorArrayOutput()
    {
        AssertExpectedDataOnCreate(_lutColorArrayInput.Length * 4, _expectedLutOutput);
        var lut = new OVRPassthroughColorLut(_lutColorArrayInput, OVRPassthroughColorLut.ColorChannels.Rgba);
        Assert.IsTrue(lut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestExpectedColor32ArrayOutput()
    {
        AssertExpectedDataOnCreate(_lutColor32ArrayInput.Length * 4, _expectedLutOutput);
        var lut = new OVRPassthroughColorLut(_lutColor32ArrayInput, OVRPassthroughColorLut.ColorChannels.Rgba);
        Assert.IsTrue(lut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestExpectedByteArrayOutput()
    {
        byte[] byteArray = GetByteInputArray();

        AssertExpectedDataOnCreate(byteArray.Length, _expectedLutOutput);
        var lut = new OVRPassthroughColorLut(byteArray, OVRPassthroughColorLut.ColorChannels.Rgba);
        Assert.IsTrue(lut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestExpectedSquareTextureOutput()
    {
        AssertExpectedDataOnCreate(_squareLutByteArrayInput.Length, _expectedSquareLutOutput);
        var squareTexture = new Texture2D(8, 8, TextureFormat.RGB24, false);
        var colors = new Color32[8 * 8];
        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = new Color32(_squareLutByteArrayInput[i * 3], _squareLutByteArrayInput[i * 3 + 1],
                _squareLutByteArrayInput[i * 3 + 2], 255);
        }

        squareTexture.SetPixels32(colors);
        squareTexture.Apply();
        var lut = new OVRPassthroughColorLut(squareTexture);
        Assert.IsTrue(lut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestIfNotInitializedAfterDispose()
    {
        var texture = GetBlankTextureOfResolution(16);
        var colorLut = new OVRPassthroughColorLut(texture);
        Assert.IsTrue(colorLut.IsValid);
        colorLut.Dispose();
        Assert.IsFalse(colorLut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromTextureFailsWithUninitializedLut()
    {
        var colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(16));
        colorLut.Dispose();
        AssertUpdateUninitializedError(() => colorLut.UpdateFrom(GetBlankTextureOfResolution(16)));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromTextureFailsWithDifferentResolution()
    {
        var colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(16));
        AssertUpdateFromResolutionError(() => colorLut.UpdateFrom(GetBlankTextureOfResolution(32)), 16);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromColorArrayFailsWithUninitializedLut()
    {
        var colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(16));
        colorLut.Dispose();
        AssertUpdateUninitializedError(() => colorLut.UpdateFrom(new Color[16 * 16 * 16]));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromColorArrayFailsWithDifferentResolution()
    {
        var colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(16));
        AssertUpdateFromResolutionError(() => colorLut.UpdateFrom(new Color[32 * 32 * 32]), 16);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromColor32ArrayFailsWithUninitializedLut()
    {
        var colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(16));
        colorLut.Dispose();
        AssertUpdateUninitializedError(() => colorLut.UpdateFrom(new Color32[16 * 16 * 16]));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromColorArray32FailsWithDifferentResolution()
    {
        var colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(16));
        AssertUpdateFromResolutionError(() => colorLut.UpdateFrom(new Color32[32 * 32 * 32]), 16);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromByteArrayFailsWithUninitializedLut()
    {
        var colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(16));
        colorLut.Dispose();
        AssertUpdateUninitializedError(() => colorLut.UpdateFrom(new byte[16 * 16 * 16 * 4]));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromByteFailsWithDifferentResolution()
    {
        var colorLut = new OVRPassthroughColorLut(GetBlankTextureOfResolution(16));
        AssertUpdateFromResolutionError(() => colorLut.UpdateFrom(new byte[32 * 32 * 32 * 4]), 16);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromTextureOutput()
    {
        var resolution = 2;
        var texture2 = new Texture2D(resolution * resolution, resolution);
        texture2.SetPixels(_lutTextureInput);
        texture2.Apply();
        AssertExpectedUpdateOutput((colorLut) => colorLut.UpdateFrom(texture2));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromColorArrayOutput()
    {
        AssertExpectedUpdateOutput((colorLut) => colorLut.UpdateFrom(_lutColorArrayInput));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromColor32ArrayOutput()
    {
        AssertExpectedUpdateOutput((colorLut) => colorLut.UpdateFrom(_lutColor32ArrayInput));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromByteArrayOutput()
    {
        AssertExpectedUpdateOutput((colorLut) => colorLut.UpdateFrom(GetByteInputArray()));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUpdateFromChangesOutput()
    {
        var resolution = 2;

        var texture = new Texture2D(resolution * resolution, resolution);
        texture.SetPixels(_lutTextureInput);
        texture.Apply();

        var colorLut = new OVRPassthroughColorLut(texture);

        var updateWasCalled = false;
        _fakeOVRPlugin84.UpdateCallback = (colorLut, data) =>
        {
            updateWasCalled = true;
            var dataSize = resolution * resolution * resolution * 4;
            byte[] managedArray = new byte[dataSize];
            Marshal.Copy(data.Buffer, managedArray, 0, dataSize);
            Assert.IsTrue(managedArray.SequenceEqual(_altExpectedLutOutput));
            return OVRPlugin.Result.Success;
        };

        colorLut.UpdateFrom(_altLutColorArrayInput);
        Assert.IsTrue(updateWasCalled);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestExpectedBufferWithColorRGBChannels()
    {
        var dataSize = _lutColorArrayInput.Length * 3;
        AssertExpectedDataOnCreate(dataSize, _expectedRGBLutOutput);
        var lut = new OVRPassthroughColorLut(_lutColorArrayInput, OVRPassthroughColorLut.ColorChannels.Rgb);
        Assert.IsTrue(lut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestExpectedBufferWithColor32RGBChannels()
    {
        var dataSize = _lutColor32ArrayInput.Length * 3;
        AssertExpectedDataOnCreate(dataSize, _expectedRGBLutOutput);
        var lut = new OVRPassthroughColorLut(_lutColor32ArrayInput, OVRPassthroughColorLut.ColorChannels.Rgb);
        Assert.IsTrue(lut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestExpectedBufferWithByteRGBChannels()
    {
        var byteArray = GetByteInputArray(3);
        var dataSize = byteArray.Length;
        AssertExpectedDataOnCreate(dataSize, _expectedRGBLutOutput);
        var lut = new OVRPassthroughColorLut(byteArray, OVRPassthroughColorLut.ColorChannels.Rgb);
        Assert.IsTrue(lut.IsValid);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestUnexpectedChannelForByteArrayIsHandled()
    {
        Assert.Throws<System.ArgumentException>(() =>
            new OVRPassthroughColorLut(GetByteInputArray(3), OVRPassthroughColorLut.ColorChannels.Rgba));
        Assert.Throws<System.ArgumentException>(() =>
            new OVRPassthroughColorLut(GetByteInputArray(4), OVRPassthroughColorLut.ColorChannels.Rgb));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestArgumentNullExceptionsThrown()
    {
        Assert.Throws<System.ArgumentNullException>(() => new OVRPassthroughColorLut((Texture2D)null));
        Assert.Throws<System.ArgumentNullException>(() =>
            new OVRPassthroughColorLut((Color[])null, OVRPassthroughColorLut.ColorChannels.Rgb));
        Assert.Throws<System.ArgumentNullException>(() =>
            new OVRPassthroughColorLut((Color32[])null, OVRPassthroughColorLut.ColorChannels.Rgb));
        Assert.Throws<System.ArgumentNullException>(() =>
            new OVRPassthroughColorLut((byte[])null, OVRPassthroughColorLut.ColorChannels.Rgb));

        var lut = new OVRPassthroughColorLut(_lutColorArrayInput, OVRPassthroughColorLut.ColorChannels.Rgb);
        Assert.Throws<System.ArgumentNullException>(() => lut.UpdateFrom((byte[])null));
        Assert.Throws<System.ArgumentNullException>(() => lut.UpdateFrom((Color[])null));
        Assert.Throws<System.ArgumentNullException>(() => lut.UpdateFrom((Color32[])null));
        Assert.Throws<System.ArgumentNullException>(() => lut.UpdateFrom((Texture2D)null));
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator PendingCreateFinishedAfterPTEnable()
    {
        var resolution = 2;
        var texture = new Texture2D(resolution * resolution, resolution);
        OVRPlugin.PassthroughColorLutData _data = default;
        _fakeOVRPlugin84.CreateCallback = (channels, resolution, data) =>
        {
            _data = data;
            return OVRPlugin.Result.Success;
        };

        var lut = new OVRPassthroughColorLut(texture);
        Assert.IsTrue(lut.IsValid, "Color LUT should be valid while creation is pending.");
        Assert.AreEqual(_data.BufferSize, 0);

        InitializeInsightPassthrough();
        var expectedSize = resolution * resolution * resolution * 4;
        Assert.AreEqual(_data.BufferSize, expectedSize);

        yield return null;
    }

    private static void InitializeInsightPassthrough()
    {
        var method = typeof(OVRManager).GetMethod("InitializeInsightPassthrough", BindingFlags.Static | BindingFlags.NonPublic);
        method.Invoke(null, null);
    }

    private void AssertExpectedUpdateOutput(System.Action<OVRPassthroughColorLut> updateAction)
    {
        var resolution = 2;

        var texture = new Texture2D(resolution * resolution, resolution);
        texture.SetPixels(_lutTextureInput);
        texture.Apply();

        var colorLut = new OVRPassthroughColorLut(texture);

        var updateWasCalled = false;
        _fakeOVRPlugin84.UpdateCallback = (colorLut, data) =>
        {
            updateWasCalled = true;
            AssertBufferDataIsAsExpected(resolution * resolution * resolution * 4, data, _expectedLutOutput);
            return OVRPlugin.Result.Success;
        };

        updateAction(colorLut);
        Assert.IsTrue(updateWasCalled);
    }

    private byte[] GetByteInputArray(int channels = 4)
    {
        var byteArray = new byte[_lutColor32ArrayInput.Length * channels];
        for (int i = 0; i < _lutColor32ArrayInput.Length; i++)
        {
            for (int c = 0; c < channels; c++)
            {
                byteArray[i * channels + c] = _lutColor32ArrayInput[i][c];
            }
        }

        return byteArray;
    }

    private void AssertUpdateFromResolutionError(System.Action updateAction, int resolution)
    {
        AssertUpdateError(updateAction, $"Can only update with the same resolution of {resolution}.");
    }

    private void AssertUpdateUninitializedError(System.Action updateAction)
    {
        AssertUpdateError(updateAction, "Can not update an uninitialized lut object.");
    }

    private void AssertUpdateError(System.Action updateAction, string expectedError)
    {
        var updateWasCalled = false;
        _fakeOVRPlugin84.UpdateCallback = (colorLut, data) =>
        {
            updateWasCalled = true;
            return OVRPlugin.Result.Success;
        };

        _myLogHandler.Expect(LogType.Error, expectedError);
        updateAction();
        Assert.IsFalse(updateWasCalled);
        Assert.IsTrue(_myLogHandler.IsHandled);
    }

    private void AssertExpectedDataOnCreate(int dataSize, byte[] expectedResult)
    {
        _fakeOVRPlugin84.CreateCallback = (channels, resolution, data) =>
        {
            AssertBufferDataIsAsExpected(dataSize, data, expectedResult);
            return OVRPlugin.Result.Success;
        };
    }

    private void AssertBufferDataIsAsExpected(int dataSize, OVRPlugin.PassthroughColorLutData data,
        byte[] expectedResult)
    {
        byte[] managedArray = new byte[dataSize];
        Marshal.Copy(data.Buffer, managedArray, 0, dataSize);
        if (managedArray.SequenceEqual(expectedResult))
        {
            Assert.Pass("Expected result and output are equal.");
        }
        else
        {
            var builder = new StringBuilder();
            builder.AppendLine("Expected: ");
            builder.AppendLine(string.Join(", ", expectedResult));
            builder.AppendLine("But got: ");
            builder.AppendLine(string.Join(", ", managedArray));
            Assert.Fail(builder.ToString());
        }
    }

    private Texture2D GetBlankTextureOfResolution(int resolution)
    {
        return new Texture2D(resolution * resolution, resolution);
    }

    private class FakeOVRPlugin66 : OVRPlugin.OVRP_1_66_0_TEST
    {
        public override OVRPlugin.Result ovrp_GetInsightPassthroughInitializationState()
        {
            return OVRPlugin.Result.Success;
        }
    }

    private class FakeOVRPlugin84 : OVRPlugin.OVRP_1_84_0_TEST
    {
        public delegate OVRPlugin.Result CreateCallbackHanlder(OVRPlugin.PassthroughColorLutChannels channels,
            uint resolution, OVRPlugin.PassthroughColorLutData data);

        public delegate OVRPlugin.Result UpdateCallbackHandler(ulong colorLut, OVRPlugin.PassthroughColorLutData data);

        public delegate OVRPlugin.Result DeleteCallbackHandler(ulong colorLut);

        public CreateCallbackHanlder CreateCallback { private get; set; }
        public UpdateCallbackHandler UpdateCallback { private get; set; }
        public DeleteCallbackHandler DeleteCallback { private get; set; }

        public override OVRPlugin.Result ovrp_CreatePassthroughColorLut(OVRPlugin.PassthroughColorLutChannels channels,
            uint resolution, OVRPlugin.PassthroughColorLutData data, out ulong colorLut)
        {
            colorLut = 0;

            if (CreateCallback != null)
            {
                return CreateCallback(channels, resolution, data);
            }

            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_UpdatePassthroughColorLut(ulong colorLut,
            OVRPlugin.PassthroughColorLutData data)
        {
            if (UpdateCallback != null)
            {
                return UpdateCallback(colorLut, data);
            }

            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_DestroyPassthroughColorLut(ulong colorLut)
        {
            if (DeleteCallback != null)
            {
                return DeleteCallback(colorLut);
            }

            return OVRPlugin.Result.Success;
        }
    }

    private class FakeOVRPlugin85 : OVRPlugin.OVRP_1_85_0_TEST
    {
        public override OVRPlugin.Result ovrp_GetPassthroughCapabilities(
            ref OVRPlugin.PassthroughCapabilities capabilityFlags)
        {
            capabilityFlags.MaxColorLutResolution = 64;
            return OVRPlugin.Result.Success;
        }
    }

    /// <summary>
    /// Works like LogAssert, but does not log expected logs to console
    /// </summary>
    private class MyLogHandler : ILogHandler
    {
        public readonly ILogHandler DefaultLogHandler = Debug.unityLogger.logHandler;

        private LogType _handleLogType;
        private string _handleLogMessage;
        public bool IsHandled { get; private set; } = true;

        public void Expect(LogType logType, string message)
        {
            _handleLogType = logType;
            _handleLogMessage = message;
            IsHandled = false;
        }

        public void LogException(System.Exception exception, Object context)
        {
            DefaultLogHandler.LogException(exception, context);
        }

        public void LogFormat(LogType logType, Object context, string format, params object[] args)
        {
            if (!IsHandled && logType == _handleLogType)
            {
                var message = string.Format(format, args);
                if (message == _handleLogMessage)
                {
                    IsHandled = true;
                }
                else
                {
                    DefaultLogHandler.LogFormat(LogType.Error, context, "Expected message: {0}, but received: {1}",
                        _handleLogMessage, message);
                }
            }
            else
            {
                DefaultLogHandler.LogFormat(logType, context, format, args);
            }
        }

        public void AssertHandled()
        {
            Assert.IsTrue(IsHandled, $"Expected to handle \"{_handleLogType}\" with message \"{_handleLogMessage}\"");
        }
    }
}

#endif
