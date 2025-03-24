namespace Robot.Runtime.Data.WorldGraph
{
    public static class WorldGraphOperation
    {
        public const string NODE_ADDED = "add/item"; // item added to world graph
        public const string NODE_REMOVED = "remove/item"; // item removed from world graph
        public const string ITEM_ADDED_TO_RECEPTACLE = "add/receptacle"; // item is contained in receptacle
        public const string ITEM_REMOVED_FROM_RECEPTACLE = "remove/receptacle"; // item is no longer contained in receptacle
        // public const string ITEM_ROOM_CHANGE = "add/update-room"; // item has been taken to another room
    }
}