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

#if OVR_INTERNAL_CODE

using System;

/// <summary>
/// Represents a string label associated with a room.
/// </summary>
public partial struct OVRRoomLabel
{
    /// <summary>
    /// Get the room label associated with this anchor.
    /// </summary>
    /// <param name="value">The label associated with the anchor.</param>
    /// <returns>Returns `True` if the label is successfully retrieved, otherwise `False`.</returns>
    public bool TryGetValue(out string value) => OVRPlugin.GetSpaceRoomLabel(Handle, out value).IsSuccess();

    /// <summary>
    /// Get the room label associated with this anchor.
    /// </summary>
    /// <remarks>
    /// If an anchor has the <see cref="OVRRoomLabel"/> component, its string value can be retrieved with this property.
    /// Note that this method throws if the getter fails; use <see cref="TryGetValue"/> to test for success.
    /// </remarks>
    /// <exception cref="InvalidOperationException">Thrown if the value could not be retrieved.</exception>
    public string Value => TryGetValue(out var value)
        ? value
        : throw new InvalidOperationException($"Unable to get the room label's value. Consider using {nameof(TryGetValue)} to test for success.");
}

#endif // OVR_INTERNAL_CODE
