using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.Utils;
using UnityEngine;

namespace States
{
    public class RetrieveEditorDetailsState : BaseInitialiseUserState
    {
        public override void begin()
        {
            base.begin();
            // use saved id so we can test reusing anchor data on server
            var savedUserId = PlayerPrefs.GetString(Consts.PlayerPref_UserId, "");
            var userId = new System.Random().NextULong();
            if (!string.IsNullOrWhiteSpace(savedUserId))
            {
                userId = ulong.Parse(savedUserId);
            }
            else
            {
                PlayerPrefs.SetString(Consts.PlayerPref_UserId, $"{userId}");
            }
            ProcessPlatformDetails(userId);
        }
    }
}