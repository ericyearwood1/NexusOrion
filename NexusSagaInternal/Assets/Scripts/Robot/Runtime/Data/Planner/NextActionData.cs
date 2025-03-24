using System.Collections.Generic;

namespace Robot.Runtime.Data.Planner
{
    public class NextActionData
    {
        public string Action;
        public bool IsReplanned;
        public string DisplayMessage;
        public List<NextActionData> ChildActions = new ();
        public PlannerActionType ActionType;
        public PlannerActionState State;
        public string Feedback;
        public int ChildActionIndex = -1;
    }
}