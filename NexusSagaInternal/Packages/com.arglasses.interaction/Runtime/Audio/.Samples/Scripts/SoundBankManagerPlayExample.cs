// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class SoundBankManagerPlayExample : MonoBehaviour
    {
        public SoundBankManager SoundBankManager;
        public TextMesh DebugText;
        public KeyCode TriggerKey = KeyCode.F;
        public KeyCode NextBankKey = KeyCode.G;
        public string EventName = "Press";

        // Update is called once per frame
        void Update()
        {
            UpdateDebugString();

            if (Input.GetKeyDown(TriggerKey))
            {
                if(SoundBankManager)
                    SoundBankManager.Play(EventName);
            }

            if (Input.GetKeyDown(NextBankKey))
            {
                NextBank();
            }
        }

        void NextBank()
        {
            if (SoundBankManager.SoundBanks.Length <= 0)
            {
                Debug.Log("SoundBankManager: Error - No sound banks defined.");
                return;
            }

            int currentBank = 0;
            for (int i = 0; i < SoundBankManager.SoundBanks.Length; ++i)
            {
                if (SoundBankManager.SoundBanks[i].BankName == SoundBankManager.ActiveBank)
                {
                    if (i == SoundBankManager.SoundBanks.Length - 1)
                    {
                        currentBank = 0;
                    }
                    else
                    {
                        currentBank = i + 1;
                    }
                    break;
                }
            }

            SoundBankManager.ActiveBank = SoundBankManager.SoundBanks[currentBank].BankName;
        }

        void UpdateDebugString()
        {
            if (!DebugText)
                return;

            string s = "Trigger Key: " + TriggerKey.ToString();
            s += '\n';
            s += "Next Bank Key: " + NextBankKey.ToString();
            s += "\n\n";

            s += "Active Bank: " + SoundBankManager.ActiveBank;

            DebugText.text = s;
        }
    }
}
