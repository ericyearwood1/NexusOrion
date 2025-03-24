using System;
using Newtonsoft.Json;
using Robot.Runtime.Data.Robot;
using SiroComms.Runtime.Messages.Server;

namespace Robot.Runtime.Messages.Server.Skills
{
    [Serializable]
    public class GripperStateMessage 
    {
        [JsonProperty("state")]
        public GripperState State;

    }
}