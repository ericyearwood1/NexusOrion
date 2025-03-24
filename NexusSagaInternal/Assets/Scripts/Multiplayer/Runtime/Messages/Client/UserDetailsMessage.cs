using SiroComms.Runtime.Messages.Client;

namespace Multiplayer.Runtime.Messages.Client
{
    public class UserDetailsMessage : IClientMessage
    {
        public const string Type = "user_details";
        public ulong UserId;
        public int Color;
        public string DisplayName;
    }
}