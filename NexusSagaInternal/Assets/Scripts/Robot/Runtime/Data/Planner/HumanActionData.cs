using System.Collections.Generic;

namespace Robot.Runtime.Data.Planner
{
    public class HumanActionData
    {
        public string Action;
        public bool IsReplanned;
        public string DisplayMessage;
        public PlannerActionType ActionType;
        public PlannerActionState State;
    }
}