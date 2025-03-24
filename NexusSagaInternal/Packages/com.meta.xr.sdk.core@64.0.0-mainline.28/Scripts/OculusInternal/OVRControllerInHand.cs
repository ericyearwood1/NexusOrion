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

using UnityEngine;

/// <summary>
/// Simple helper script for controller in hand interactions.
/// could be used for poke limiting etc.
/// </summary>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_controller_in_hand")]
public class OVRControllerInHand : MonoBehaviour
{
    /// <summary>
    /// The root GameObject that represents the controller.
    /// </summary>
    public Transform m_controllerAnchor;

    /// <summary>
    /// The root GameObject that represents the hand.
    /// </summary>
    public Transform m_handAnchor;

    /// <summary>
    /// Which hand is this for
    /// </summary>
    public OVRInput.Hand m_hand;
}
