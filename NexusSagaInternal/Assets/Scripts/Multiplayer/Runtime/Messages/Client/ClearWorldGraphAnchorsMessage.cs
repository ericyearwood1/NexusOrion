using SiroComms.Runtime.Messages.Client;

namespace Multiplayer.Runtime.Messages.Client
{
    public class ClearWorldGraphAnchorsMessage : IClientMessage
    {
        public const string Type = "clear_world_graph_anchors";
        
        public string room;

        public ClearWorldGraphAnchorsMessage(string room_id)
        {
            this.room = room_id;
        }
    }
}