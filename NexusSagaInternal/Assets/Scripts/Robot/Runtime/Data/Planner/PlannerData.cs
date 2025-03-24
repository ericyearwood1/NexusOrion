namespace Robot.Runtime.Data.Planner
{
    public class PlannerData
    {
        public PlannerState State { get; set; } = PlannerState.ReadyForInstruction; // @TODO remove once stub request is working
        public NextActionData CurrentAction { get; set; }
        public HumanActionData humanActionData { get; set;}

    }
}