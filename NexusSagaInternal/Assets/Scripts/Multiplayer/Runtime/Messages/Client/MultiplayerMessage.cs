using System;

namespace Multiplayer.Runtime.Messages.Client
{
    [Serializable]
    public class MultiplayerMessage
    {
        public readonly string room;
        
        public MultiplayerMessage(string roomId)
        {
            room = roomId;
        }
    }
}