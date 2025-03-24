using Data;
using Meta.XR.MRUtilityKit;
using Prime31.StateKit;

namespace States
{
    public class LoadOVRRoomDataState : SKState<AppData>
    {
        public override void begin()
        {
            base.begin();
            MRUK.Instance?.RegisterSceneLoadedCallback(OnSceneLoaded);
        }

        private void OnSceneLoaded()
        {
            throw new System.NotImplementedException();
        }

        public override void update(float deltaTime)
        {
        }
    }
}