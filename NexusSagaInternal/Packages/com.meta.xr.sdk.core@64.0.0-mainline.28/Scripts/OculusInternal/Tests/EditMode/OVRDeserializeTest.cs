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

using System;
using System.Runtime.InteropServices;
using NUnit.Framework;

//-------------------------------------------------------------------------------------
/// <summary>
/// Tests for the OVRDeserialize class. The magic numbers in the file were kept to make sure the previous behaviour
/// was properly replicated
/// </summary>
public static class OVRDeserializeTest
{
    [Test]
    public static void TestSpaceQueryCompleteDataDeserialize()
    {
        var originalData = new OVRDeserialize.SpaceQueryCompleteData
        {
            RequestId = 999,
            Result = 111
        };

        // Validate struct size, if new fields are added this test should be updated
        Assert.AreEqual(16, Marshal.SizeOf(typeof(OVRDeserialize.SpaceQueryCompleteData)));

        var byteArray = StructureToByteArray(originalData);
        Assert.AreEqual(originalData.RequestId, BitConverter.ToUInt64(byteArray, 0));
        Assert.AreEqual(originalData.Result, BitConverter.ToInt32(byteArray, 8));

        var deserializedData = OVRDeserialize.ByteArrayToStructure<OVRDeserialize.SpaceQueryCompleteData>(byteArray);

        Assert.AreEqual(originalData.RequestId, deserializedData.RequestId);
        Assert.AreEqual(originalData.Result, deserializedData.Result);
    }

    [Test]
    public static void TestSpatialAnchorCreateCompleteDataDeserialize()
    {
        var originalData =
            new OVRDeserialize.SpatialAnchorCreateCompleteData
            {
                RequestId = 999,
                Result = 888,
                Space = 77777,
                Uuid = Guid.NewGuid()
            };

        // Validate struct size, if new fields are added this test should be updated
        Assert.AreEqual(40, Marshal.SizeOf(typeof(OVRDeserialize.SpatialAnchorCreateCompleteData)));

        var byteArray = StructureToByteArray(originalData);

        Assert.AreEqual(originalData.RequestId, BitConverter.ToUInt64(byteArray, 0));
        Assert.AreEqual(originalData.Result, BitConverter.ToUInt32(byteArray, 8));
        Assert.AreEqual(originalData.Space, BitConverter.ToUInt64(byteArray, 16));
        Assert.AreEqual(originalData.Uuid, GuidFromBytes(byteArray, 24));

        var data = OVRDeserialize.ByteArrayToStructure<OVRDeserialize.SpatialAnchorCreateCompleteData>(byteArray);

        Assert.AreEqual(originalData.RequestId, data.RequestId);
        Assert.AreEqual(originalData.Result, data.Result);
        Assert.AreEqual(originalData.Space, data.Space);
        Assert.AreEqual(originalData.Uuid, data.Uuid);
    }

    [Test]
    public static void TestSpaceSaveCompleteDataDeserialize()
    {
        var originalData =
            new OVRDeserialize.SpaceSaveCompleteData
            {
                RequestId = 999,
                Space = 77777,
                Result = 888,
                Uuid = Guid.NewGuid()
            };

        // Validate struct size, if new fields are added this test should be updated
        Assert.AreEqual(40, Marshal.SizeOf(typeof(OVRDeserialize.SpaceSaveCompleteData)));

        var byteArray = StructureToByteArray(originalData);

        Assert.AreEqual(originalData.RequestId, BitConverter.ToUInt64(byteArray, 0));
        Assert.AreEqual(originalData.Space, BitConverter.ToUInt64(byteArray, 8));
        Assert.AreEqual(originalData.Result, BitConverter.ToUInt32(byteArray, 16));
        Assert.AreEqual(originalData.Uuid, GuidFromBytes(byteArray, 20));

        var data = OVRDeserialize.ByteArrayToStructure<OVRDeserialize.SpaceSaveCompleteData>(byteArray);

        Assert.AreEqual(originalData.RequestId, data.RequestId);
        Assert.AreEqual(originalData.Space, data.Space);
        Assert.AreEqual(originalData.Result, data.Result);
        Assert.AreEqual(originalData.Uuid, data.Uuid);
    }


    [Test]
    public static void TestSpaceEraseCompleteDataDeserialize()
    {
        var originalData =
            new OVRDeserialize.SpaceEraseCompleteData
            {
                RequestId = 999,
                Result = 888,
                Uuid = Guid.NewGuid(),
                Location = OVRPlugin.SpaceStorageLocation.Cloud
            };

        // Validate struct size, if new fields are added this test should be updated
        Assert.AreEqual(32, Marshal.SizeOf(typeof(OVRDeserialize.SpaceEraseCompleteData)));

        var byteArray = StructureToByteArray(originalData);

        Assert.AreEqual(originalData.RequestId, BitConverter.ToUInt64(byteArray, 0));
        Assert.AreEqual(originalData.Result, BitConverter.ToUInt32(byteArray, 8));
        Assert.AreEqual(originalData.Uuid, GuidFromBytes(byteArray, 12));
        Assert.AreEqual(originalData.Location, (OVRPlugin.SpaceStorageLocation)BitConverter.ToUInt32(byteArray, 28));

        var data = OVRDeserialize.ByteArrayToStructure<OVRDeserialize.SpaceEraseCompleteData>(byteArray);

        Assert.AreEqual(originalData.RequestId, data.RequestId);
        Assert.AreEqual(originalData.Result, data.Result);
        Assert.AreEqual(originalData.Uuid, data.Uuid);
        Assert.AreEqual(originalData.Location, data.Location);
    }

    [Test]
    public static void TestSpaceSetComponentStatusCompleteDataDeserialize()
    {
        var originalData =
            new OVRDeserialize.SpaceSetComponentStatusCompleteData
            {
                RequestId = 999,
                Result = 888,
                Space = 77777,
                Uuid = Guid.NewGuid(),
                ComponentType = OVRPlugin.SpaceComponentType.Storable,
                Enabled = 1
            };

        // Validate struct size, if new fields are added this test should be updated
        Assert.AreEqual(48, Marshal.SizeOf(typeof(OVRDeserialize.SpaceSetComponentStatusCompleteData)));

        var byteArray = StructureToByteArray(originalData);

        Assert.AreEqual(originalData.RequestId, BitConverter.ToUInt64(byteArray, 0));
        Assert.AreEqual(originalData.Result, BitConverter.ToUInt32(byteArray, 8));
        Assert.AreEqual(originalData.Space, BitConverter.ToUInt64(byteArray, 16));
        Assert.AreEqual(originalData.Uuid, GuidFromBytes(byteArray, 24));
        Assert.AreEqual(originalData.ComponentType, (OVRPlugin.SpaceComponentType)BitConverter.ToUInt32(byteArray, 40));
        Assert.AreEqual(originalData.Enabled, BitConverter.ToUInt32(byteArray, 44));

        var data = OVRDeserialize.ByteArrayToStructure<OVRDeserialize.SpaceSetComponentStatusCompleteData>(byteArray);

        Assert.AreEqual(originalData.RequestId, data.RequestId);
        Assert.AreEqual(originalData.Result, data.Result);
        Assert.AreEqual(originalData.Space, data.Space);
        Assert.AreEqual(originalData.Uuid, data.Uuid);
        Assert.AreEqual(originalData.ComponentType, data.ComponentType);
        Assert.AreEqual(originalData.Enabled, data.Enabled);
    }

    [Test]
    public static void TestSpatialEntityShareSpaceResultHeaderDataDeserialize()
    {
        var originalData =
            new OVRDeserialize.SpaceShareResultData
            {
                RequestId = 999,
                Result = 888,
            };

        // Validate struct size, if new fields are added this test should be updated
        Assert.AreEqual(16, Marshal.SizeOf(typeof(OVRDeserialize.SpaceShareResultData)));

        var byteArray = StructureToByteArray(originalData);

        Assert.AreEqual(originalData.RequestId, BitConverter.ToUInt64(byteArray, 0));
        Assert.AreEqual(originalData.Result, BitConverter.ToUInt32(byteArray, 8));

        var data = OVRDeserialize.ByteArrayToStructure<OVRDeserialize.SpaceShareResultData>(byteArray);

        Assert.AreEqual(originalData.RequestId, data.RequestId);
        Assert.AreEqual(originalData.Result, data.Result);
    }

    private static readonly byte[] _guidBuffer = new byte[16];

    private static Guid GuidFromBytes(byte[] bytes, int offset)
    {
        Buffer.BlockCopy(bytes, offset, _guidBuffer, 0, 16);
        return new Guid(_guidBuffer);
    }

    private static byte[] StructureToByteArray(object obj)
    {
        int len = Marshal.SizeOf(obj);

        byte[] arr = new byte[len];

        IntPtr ptr = Marshal.AllocHGlobal(len);

        Marshal.StructureToPtr(obj, ptr, true);

        Marshal.Copy(ptr, arr, 0, len);

        Marshal.FreeHGlobal(ptr);

        return arr;
    }
}

#endif // OVRPLUGIN_TESTING
