using System.Collections.Generic;
using Robot.Runtime.Data;
using Robot.Runtime.Messages.Server.HumanActivity;
using Robot.Runtime.ServiceHandlers;
using SiroComms.Runtime.Messages.Server;
using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public class StubHumanActivityService : StubService
    {
        private const float MinTimeBetweenActivities = 5f;
        private float _timeSinceLastActivity = 0f;

        private string[] _activities =
        {
            "Holding Cup",
            "Walking",
            "Dancing",
            "Standing"
        };

        private int _currentIndex;

        public StubHumanActivityService(Queue<string> messageQueue) : base(messageQueue)
        {
        }

        public override void Tick()
        {
            base.Tick();
            _timeSinceLastActivity += Time.deltaTime;
            if (_timeSinceLastActivity < MinTimeBetweenActivities) return;
            if (Random.value < 0.5f) return;
            _timeSinceLastActivity = 0;
            ShowNextActivity();
        }

        private void ShowNextActivity()
        {
            var activity = _activities[_currentIndex];
            _currentIndex++;
            _currentIndex %= _activities.Length;
            var data = new HumanActivityMessage
            {
                Activity = activity
            };
            SendMessage(HumanActivityMessageHandler.TYPE, HumanActivityMessageHandler.HUMAN_ACTIVITY,
                data);
        }
    }
}