using System;
using System.Collections.Generic;

namespace Multiplayer.Runtime.Data
{
    [Serializable]
    public class RoomData
    {
        public string RoomId;
        public List<User> Users;
    }
}