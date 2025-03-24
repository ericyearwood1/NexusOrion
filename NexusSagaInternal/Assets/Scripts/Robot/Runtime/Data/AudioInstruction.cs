using System;

namespace Robot.Runtime.Data
{
    [Serializable]
    public class AudioInstruction : Instruction
    {
        public AudioInstruction(string audioBytes, string userId) : base(audioBytes, "audio", userId)
        {
        }
    }
}