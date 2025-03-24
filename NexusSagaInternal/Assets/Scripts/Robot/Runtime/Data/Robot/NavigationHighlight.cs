using Newtonsoft.Json;
using UnityEngine;

namespace Robot.Runtime.Data.Robot
{
    public class NavigationHighlight
    {
        [JsonProperty("target")]
        public string Target;

        [JsonProperty("position")]
        public Vector3 Position;
    }
}