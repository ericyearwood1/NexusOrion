// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;

[AttributeUsage(AttributeTargets.Class)]
public sealed class AssetPathAttribute : Attribute
{
    public string Path { get; }

    public AssetPathAttribute(string filePath)
    {
        Path = filePath;
    }
}
