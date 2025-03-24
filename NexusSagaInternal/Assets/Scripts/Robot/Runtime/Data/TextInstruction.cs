using System;

namespace Robot.Runtime.Data
{
    [Serializable]
    public class TextInstruction : Instruction
    {
        public TextInstruction(string instruction, string userId) : base(instruction, "text", userId)
        {
        }
    }
}