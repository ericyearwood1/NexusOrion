namespace Robot.Runtime.Data.Planner
{
    public enum PlannerState
    {
        None,
        ReadyForInstruction,
        SendingInstruction,
        ReceivedInstruction,
        PlannerBusy,
        InstructionComplete,
        ServiceUnavailable,
        Replanning,
        HumanActivity
    }
}