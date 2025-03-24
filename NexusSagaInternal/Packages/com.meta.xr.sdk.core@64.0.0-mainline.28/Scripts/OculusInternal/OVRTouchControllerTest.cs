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
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

public class OVRTouchControllerTest : MonoBehaviour
{
    public class BoolMonitor
    {
        public delegate bool BoolGenerator();

        private string m_name = "";
        private BoolGenerator m_generator;
        private bool m_prevValue = false;
        private bool m_currentValue = false;
        private bool m_currentValueRecentlyChanged = false;
        private float m_displayTimeout = 0.0f;
        private float m_displayTimer = 0.0f;

        public BoolMonitor(string name, BoolGenerator generator, float displayTimeout = 0.5f)
        {
            m_name = name;
            m_generator = generator;
            m_displayTimeout = displayTimeout;
        }

        public void Update()
        {
            m_prevValue = m_currentValue;
            m_currentValue = m_generator();

            if (m_currentValue != m_prevValue)
            {
                m_currentValueRecentlyChanged = true;
                m_displayTimer = m_displayTimeout;
            }

            if (m_displayTimer > 0.0f)
            {
                m_displayTimer -= Time.deltaTime;

                if (m_displayTimer <= 0.0f)
                {
                    m_currentValueRecentlyChanged = false;
                    m_displayTimer = 0.0f;
                }
            }
        }

        public void AppendToStringBuilder(ref StringBuilder sb)
        {
            sb.Append(m_name);

            if (m_currentValue && m_currentValueRecentlyChanged)
                sb.Append(": *True*\n");
            else if (m_currentValue)
                sb.Append(":  True \n");
            else if (!m_currentValue && m_currentValueRecentlyChanged)
                sb.Append(": *False*\n");
            else if (!m_currentValue)
                sb.Append(":  False \n");
        }
    }

    public Text uiText;
    public AudioClip audioClip;
    private List<BoolMonitor> monitors;
    private StringBuilder data;
    private OVRHapticsClip hapticsClip;

    void Start()
    {
        if (uiText != null)
        {
            uiText.supportRichText = false;
        }

        data = new StringBuilder(2048);

        monitors = new List<BoolMonitor>()
        {
            new BoolMonitor("Button PriThumbstick", () => OVRInput.Get(OVRInput.Button.PrimaryThumbstick)),
            new BoolMonitor("Button SecThumbstick", () => OVRInput.Get(OVRInput.Button.SecondaryThumbstick)),
            new BoolMonitor("Button Menu", () => OVRInput.Get(OVRInput.Button.Start)),

            new BoolMonitor("Button One", () => OVRInput.Get(OVRInput.Button.One)),
            new BoolMonitor("Button Two", () => OVRInput.Get(OVRInput.Button.Two)),
            new BoolMonitor("Button Three", () => OVRInput.Get(OVRInput.Button.Three)),
            new BoolMonitor("Button Four", () => OVRInput.Get(OVRInput.Button.Four)),

            new BoolMonitor("Touch One", () => OVRInput.Get(OVRInput.Touch.One)),
            new BoolMonitor("Touch Two", () => OVRInput.Get(OVRInput.Touch.Two)),
            new BoolMonitor("Touch Three", () => OVRInput.Get(OVRInput.Touch.Three)),
            new BoolMonitor("Touch Four", () => OVRInput.Get(OVRInput.Touch.Four)),

            new BoolMonitor("Touch PriIndexTrigger", () => OVRInput.Get(OVRInput.Touch.PrimaryIndexTrigger)),
            new BoolMonitor("Touch PriThumbstick", () => OVRInput.Get(OVRInput.Touch.PrimaryThumbstick)),
            new BoolMonitor("Touch PriThumbRest", () => OVRInput.Get(OVRInput.Touch.PrimaryThumbRest)),
            new BoolMonitor("Touch PriTouchpad", () => OVRInput.Get(OVRInput.Touch.PrimaryTouchpad)),

            new BoolMonitor("Touch SecIndexTrigger", () => OVRInput.Get(OVRInput.Touch.SecondaryIndexTrigger)),
            new BoolMonitor("Touch SecThumbstick", () => OVRInput.Get(OVRInput.Touch.SecondaryThumbstick)),
            new BoolMonitor("Touch SecThumbRest", () => OVRInput.Get(OVRInput.Touch.SecondaryThumbRest)),
            new BoolMonitor("Touch SecTouchpad", () => OVRInput.Get(OVRInput.Touch.SecondaryTouchpad)),

            new BoolMonitor("Near PriIndexTrigger", () => OVRInput.Get(OVRInput.NearTouch.PrimaryIndexTrigger)),
            new BoolMonitor("Near PriThumbButtons", () => OVRInput.Get(OVRInput.NearTouch.PrimaryThumbButtons)),

            new BoolMonitor("Near SecIndexTrigger", () => OVRInput.Get(OVRInput.NearTouch.SecondaryIndexTrigger)),
            new BoolMonitor("Near SecThumbButtons", () => OVRInput.Get(OVRInput.NearTouch.SecondaryThumbButtons)),
        };
    }

    private float[] m_hapticsBuffer = new float[(int)OVRPlugin.HapticsConstants.MaxSamples];

    static string prevConnected = "";

    static BoolMonitor controllers = new BoolMonitor("Controllers Changed",
        () => { return OVRInput.GetConnectedControllers().ToString() != prevConnected; });

    void Update()
    {
        data.Length = 0;

        OVRInput.Controller activeController = OVRInput.GetActiveController();

        string activeControllerName = activeController.ToString();
        data.AppendFormat("Active: {0}\n", activeControllerName);

        string connectedControllerNames = OVRInput.GetConnectedControllers().ToString();
        data.AppendFormat("Connected: {0}\n", connectedControllerNames);

        data.AppendFormat("PrevConnected: {0}\n", prevConnected);

        data.AppendFormat("SystemHeadsetType: {0}\n", OVRPlugin.GetSystemHeadsetType());

        data.AppendFormat("LeftHandProfile: {0}\n", OVRInput.GetCurrentInteractionProfile(OVRInput.Hand.HandLeft));
        data.AppendFormat("RightHandProfile: {0}\n", OVRInput.GetCurrentInteractionProfile(OVRInput.Hand.HandRight));

        controllers.Update();
        controllers.AppendToStringBuilder(ref data);
        prevConnected = connectedControllerNames;

        Vector3 leftPos = OVRInput.GetLocalControllerPosition(activeController & OVRInput.Controller.LTouch);
        data.AppendFormat("L Position: ({0:F2}, {1:F2}, {2:F2})\n", leftPos.x, leftPos.y, leftPos.z);
        Quaternion leftRot = OVRInput.GetLocalControllerRotation(activeController & OVRInput.Controller.LTouch);
        data.AppendFormat("L Orientation: ({0:F2}, {1:F2}, {2:F2}, {3:F2})\n", leftRot.x, leftRot.y, leftRot.z,
            leftRot.w);
        Vector3 rightPos = OVRInput.GetLocalControllerPosition(activeController & OVRInput.Controller.RTouch);
        data.AppendFormat("R Position: ({0:F2}, {1:F2}, {2:F2})\n", rightPos.x, rightPos.y, rightPos.z);
        Quaternion rightRot = OVRInput.GetLocalControllerRotation(activeController & OVRInput.Controller.RTouch);
        data.AppendFormat("R Orientation: ({0:F2}, {1:F2}, {2:F2}, {3:F2})\n", rightRot.x, rightRot.y, rightRot.z,
            rightRot.w);

        data.AppendFormat("PrimaryHandTrigger: {0}\n", OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger));
        data.AppendFormat("PrimaryIndexTrigger: {0}\n", OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger));
        data.AppendFormat("PrimaryIndexTriggerCurl: {0}\n", OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTriggerCurl));
        data.AppendFormat("PrimaryIndexTriggerSlide: {0}\n", OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTriggerSlide));
        data.AppendFormat("PrimaryIndexTriggerForce: {0}\n", OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTriggerForce));
        data.AppendFormat("PrimaryThumbRestForce: {0}\n", OVRInput.Get(OVRInput.Axis1D.PrimaryThumbRestForce));
        data.AppendFormat("PrimaryStylusForce: {0}\n", OVRInput.Get(OVRInput.Axis1D.PrimaryStylusForce));

        data.AppendFormat("SecondaryHandTrigger: {0}\n", OVRInput.Get(OVRInput.Axis1D.SecondaryHandTrigger));
        data.AppendFormat("SecondaryIndexTrigger: {0}\n", OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTrigger));
        data.AppendFormat("SecondaryIndexTriggerCurl: {0}\n", OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTriggerCurl));
        data.AppendFormat("SecondaryIndexTriggerSlide: {0}\n",
            OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTriggerSlide));
        data.AppendFormat("SecondaryIndexTriggerForce: {0}\n",
            OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTriggerForce));
        data.AppendFormat("SecondaryThumbRestForce: {0}\n", OVRInput.Get(OVRInput.Axis1D.SecondaryThumbRestForce));
        data.AppendFormat("SecondaryStylusForce: {0}\n", OVRInput.Get(OVRInput.Axis1D.SecondaryStylusForce));

        Vector2 primaryThumbstick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick);
        data.AppendFormat("PriThumbstick: ({0:F2}, {1:F2})\n", primaryThumbstick.x, primaryThumbstick.y);
        Vector2 primaryTouchpad = OVRInput.Get(OVRInput.Axis2D.PrimaryTouchpad);
        data.AppendFormat("PriTouchpad: ({0:F2}, {1:F2})\n", primaryTouchpad.x, primaryTouchpad.y);

        Vector2 secondaryThumbstick = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick);
        data.AppendFormat("SecThumbstick: ({0:F2}, {1:F2})\n", secondaryThumbstick.x, secondaryThumbstick.y);
        Vector2 secondaryTouchpad = OVRInput.Get(OVRInput.Axis2D.SecondaryTouchpad);
        data.AppendFormat("SecTouchpad: ({0:F2}, {1:F2})\n", secondaryTouchpad.x, secondaryTouchpad.y);

        int samplesConsumed = 0;
        float leftSampleRateHz = OVRInput.GetControllerSampleRateHz(OVRInput.Controller.LTouch);
        float rightSampleRateHz = OVRInput.GetControllerSampleRateHz(OVRInput.Controller.RTouch);
        data.AppendFormat("L SampleRateHz: {0}\n", leftSampleRateHz);
        data.AppendFormat("R SampleRateHz: {0}\n", rightSampleRateHz);

        if (OVRInput.Get(OVRInput.Button.Two))
        {
            OVRInput.HapticsPcmVibration hapticsVibration;
            hapticsVibration.SamplesCount = m_hapticsBuffer.Length;
            hapticsVibration.Samples = m_hapticsBuffer;
            hapticsVibration.SampleRateHz = OVRInput.GetControllerSampleRateHz(OVRInput.Controller.RTouch);
            hapticsVibration.Append = true;

            for (int i = 0; i < hapticsVibration.SamplesCount; i++)
            {
                hapticsVibration.Samples[i] = Mathf.Sin(157.0f * (i / 1000.0f) * 6.283184f);
            }

            samplesConsumed = OVRInput.SetControllerHapticsPcm(hapticsVibration, OVRInput.Controller.RTouch);
        }

        data.AppendFormat("SamplesConsumed: {0}\n", samplesConsumed);

        if (OVRInput.Get(OVRInput.Button.Four))
        {
            OVRInput.HapticsAmplitudeEnvelopeVibration hapticsVibration;
            hapticsVibration.SamplesCount = m_hapticsBuffer.Length;
            hapticsVibration.Samples = m_hapticsBuffer;
            hapticsVibration.Duration = 0.5f;

            for (int i = 0; i < hapticsVibration.SamplesCount; i++)
            {
                hapticsVibration.Samples[i] = Mathf.Sin(157.0f * (i / 1000.0f) * 6.283184f);
            }

            OVRInput.SetControllerHapticsAmplitudeEnvelope(hapticsVibration, OVRInput.Controller.LTouch);
        }

        if (OVRInput.Get(OVRInput.Button.One))
        {
            OVRInput.SetControllerLocalizedVibration(OVRInput.HapticsLocation.Hand, 1, 1, OVRInput.Controller.RTouch);
        }

        if (OVRInput.Get(OVRInput.Axis1D.SecondaryThumbRestForce) > 0.1f)
        {
            OVRInput.SetControllerLocalizedVibration(OVRInput.HapticsLocation.Thumb, 1,
                OVRInput.Get(OVRInput.Axis1D.SecondaryThumbRestForce), OVRInput.Controller.RTouch);
        }

        if (OVRInput.Get(OVRInput.Button.Three))
        {
            OVRInput.SetControllerLocalizedVibration(OVRInput.HapticsLocation.Hand, 1, 1, OVRInput.Controller.LTouch);
        }

        if (OVRInput.Get(OVRInput.Axis1D.PrimaryThumbRestForce) > 0.1f)
        {
            OVRInput.SetControllerLocalizedVibration(OVRInput.HapticsLocation.Thumb, 1,
                OVRInput.Get(OVRInput.Axis1D.PrimaryThumbRestForce), OVRInput.Controller.LTouch);
        }

        if (OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger) > 0.1f)
        {
            float primaryIndexVal = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger);
            OVRInput.SetControllerLocalizedVibration(OVRInput.HapticsLocation.Index, 1, primaryIndexVal,
                OVRInput.Controller.LTouch);
        }

        if (OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTrigger) > 0.1f)
        {
            float secondaryIndexVal = OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTrigger);
            OVRInput.SetControllerLocalizedVibration(OVRInput.HapticsLocation.Index, 1, secondaryIndexVal,
                OVRInput.Controller.RTouch);
        }

        if (OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger) > 0.1f)
        {
            hapticsClip = new OVRHapticsClip(audioClip);
            OVRHaptics.RightChannel.Preempt(hapticsClip);
        }

        for (int i = 0; i < monitors.Count; i++)
        {
            monitors[i].Update();
            monitors[i].AppendToStringBuilder(ref data);
        }

        if (uiText != null)
        {
            uiText.text = data.ToString();
        }
    }
}
#endif
