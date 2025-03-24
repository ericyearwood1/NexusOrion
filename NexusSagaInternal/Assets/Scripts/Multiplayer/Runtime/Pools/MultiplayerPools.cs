using Multiplayer.Runtime.Config;
using Multiplayer.Runtime.View;
using UnityEngine;

namespace Multiplayer.Runtime.Pools
{
    public class MultiplayerPools
    {
        public UserPool<UserEntityView> HeadPool { get; private set; }
        // public UserPool<UserEntityView> LeftHandPool { get; private set; }
        // public UserPool<UserEntityView> RightHandPool { get; private set; }
        public UserPool<UserNameDisplay> UserDisplayPool { get; private set; }

        public void Initialise(MultiplayerPoolConfig poolConfig, Transform container = null)
        {
            HeadPool = new UserPool<UserEntityView>();
            HeadPool.Initialise(poolConfig.HeadPrefab, poolConfig.PoolSize, container);
            // LeftHandPool = new UserPool<UserEntityView>();
            // LeftHandPool.Initialise(poolConfig.LeftHandPrefab, poolConfig.PoolSize, container);
            // RightHandPool = new UserPool<UserEntityView>();
            // RightHandPool.Initialise(poolConfig.RightHandPrefab, poolConfig.PoolSize, container);
            UserDisplayPool = new UserPool<UserNameDisplay>();
            UserDisplayPool.Initialise(poolConfig.UserNameDisplayPrefab, poolConfig.PoolSize, container);
        }
    }
}