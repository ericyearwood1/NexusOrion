using System;
using Multiplayer.Runtime.Data;
using SiroComms.Runtime.Messages.Client;

namespace Multiplayer.Runtime.Messages.Client
{
    [Serializable]
    public class JoinRoomMessage : IClientMessage
    {
        public const string Type = "join_room";
        
        public string room;
        public User user;

        public JoinRoomMessage(string room_id, User user)
        {
            this.room = room_id;
            this.user = user;
        }
    }
}