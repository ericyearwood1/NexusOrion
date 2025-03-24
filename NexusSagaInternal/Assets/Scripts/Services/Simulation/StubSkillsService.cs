using System.Collections.Generic;
using System.Threading.Tasks;
using Robot.Runtime.Data;
using Robot.Runtime.Data.Robot;
using Robot.Runtime.Messages.Server.Skills;
using Robot.Runtime.ServiceHandlers;
using SiroComms.Runtime.Services.Utils;
using UnityEditor;
using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public class StubSkillsService : StubService
    {
        private bool _isFakeRobotActive;
        private RobotSimulation _robotSim;
        private readonly RobotArmSimulation _robotArmSim;
        private readonly Transform _robotFrame;
        private readonly Transform _robotHome;

        public StubSkillsService(Queue<string> messageQueue, Transform robotFrame, Transform robotHome) : base(messageQueue)
        {
            _robotHome = robotHome;
            _robotFrame = robotFrame;
            _robotSim = new RobotSimulation(Vector3.zero);
            _robotArmSim = new RobotArmSimulation(new Vector3(0, 0.78f, 0));
        }

        public bool IsRobotActive => _isFakeRobotActive;

        public override void Tick()
        {
            base.Tick();
            if (!_isFakeRobotActive) return;
            _robotSim.Tick();
            _robotArmSim.Tick();
            SendPoseMessage();
            SendEEPoseMessage();
        }

        public async Task OpenDrawer(string actionId, string label, Vector3 position)
        {
            _isFakeRobotActive = true;
            var localRobotPoint = _robotHome.TransformPoint(position);
            Debug.Log($"Open drawer {localRobotPoint} | {position}");
            SendFoundObjectMessage(actionId, label, position);
            SendGripperStateUpdate(GripperState.Open);
            await SendEEToPoint(position);
            SendGripperStateUpdate(GripperState.Holding);
            await SendEEToDefaultPosition();
            SendGripperStateUpdate(GripperState.Open);
            await Task.Delay(1000);
            SendGripperStateUpdate(GripperState.Closed);
            _isFakeRobotActive = false;
        }

        public async Task CloseDrawer(string actionId, string label, Vector3 position)
        {
            var worldPos = _robotFrame.TransformPoint(_robotArmSim.CurrentPosition);
            var target = _robotHome.InverseTransformPoint(worldPos);
            Debug.Log($"CloseDrawer | {_robotArmSim.CurrentPosition} | {target}");
            _isFakeRobotActive = true;
            SendFoundObjectMessage(actionId, label, target);
            SendGripperStateUpdate(GripperState.Open);
            await Task.Delay(1000);
            SendGripperStateUpdate(GripperState.Holding);
            await SendEEToPoint(position);
            SendGripperStateUpdate(GripperState.Open);
            await Task.Delay(1000);
            SendGripperStateUpdate(GripperState.Closed);
            await SendEEToDefaultPosition();
            _isFakeRobotActive = false;
        }

        public async Task PickUpItem(string actionId, string label, Vector3 position, int numberOptions)
        {
            Debug.Log($"PickUpItem {position}");
            _isFakeRobotActive = true;
            await Task.Delay(500);
            SendDetectingObjectsMessage(actionId, label, position, numberOptions);
            await Task.Delay(2000);
            SendFoundObjectMessage(actionId, label, position);
            SendGripperStateUpdate(GripperState.Open);
            await Task.Delay(1000);
            await SendEEToPoint(position);
            SendGripperStateUpdate(GripperState.Holding);
            await SendEEToDefaultPosition();
            _isFakeRobotActive = false;
        }

        public async Task PlaceItem(string actionId, Vector3 position,  Quaternion targetOrientation=default, Vector3 targetEuler=default)
        {
            _isFakeRobotActive = true;
            await Task.Delay(500);
            SendPlaceLocationObjectMessage(actionId, position, targetOrientation, targetEuler);
            await Task.Delay(2000);
            
            await SendEEToPoint(position);
            await Task.Delay(2000);
            SendGripperStateUpdate(GripperState.Open);
            await Task.Delay(2000);
            SendGripperStateUpdate(GripperState.Closed);
            await SendEEToDefaultPosition();
            _isFakeRobotActive = false;
            _robotSim.ResetState();
            _robotArmSim.ResetState();
        }

        public async Task NavigateTo(string actionId, string label, Waypoint target)
        {
            _isFakeRobotActive = true;
            var position = target.Get3DPosition();
            _robotSim.NavigateToPosition(position, target.Yaw);
            SendNavigationHighlightMessage(actionId, label, position);
            while (_robotSim.State != SimulationState.Success)
            {
                await Task.Yield();
            }

            _isFakeRobotActive = false;
            _robotSim.ResetState();
            _robotArmSim.ResetState();
        }
        
        private async Task SendEEToPoint(Vector3 position)
        {
            _isFakeRobotActive = true;
            var worldPosition = _robotHome.TransformPoint(position);
            _robotArmSim.NavigateToPosition(_robotFrame.InverseTransformPoint(worldPosition));
            while (_robotArmSim.State != SimulationState.Success)
            {
                await Task.Yield();
            }

            _isFakeRobotActive = false;
            _robotSim.ResetState();
            _robotArmSim.ResetState();
        }
        
        private async Task SendEEToDefaultPosition()
        {
            _isFakeRobotActive = true;
            _robotArmSim.ReturnToHome();
            while (_robotArmSim.State != SimulationState.Success)
            {
                await Task.Yield();
            }

            _isFakeRobotActive = false;
            _robotSim.ResetState();
            _robotArmSim.ResetState();
        }

        private void SendNavigationHighlightMessage(string actionId, string label, Vector3 targetPosition)
        {
            var position = targetPosition;
            var data = new NavigationHighlightMessage
            {
                ActionId = actionId,
                Position = RobotConversion.Vector3ToRobotPose(position),
                Target = label
            };
            SendMessage(SkillsMessageHandler.TYPE, SkillsMessageHandler.NAVIGATION_HIGHLIGHT, data);
        }

        private void SendPlaceLocationObjectMessage(string actionId, Vector3 position, Quaternion targetOrientation=default, Vector3 targetEuler=default)
        {
            Debug.Log($"SendPlaceLocationObjectMessage {position} | {targetOrientation}");
            var data = new PlaceLocationMessage
            {
                ActionId = actionId,
                Position = RobotConversion.Vector3ToRobotPose(position),
                TargetOrientation = RobotConversion.QuaternionToRobotPose(targetOrientation),
                TargetEuler = targetEuler
            };
            Debug.Log($"SendPlaceLocationObjectMessage {data.Position} | {data.TargetOrientation} | {data.TargetEuler}");
            SendMessage(SkillsMessageHandler.TYPE, SkillsMessageHandler.PLACE_LOCATION, data);
        }

        private void SendGripperStateUpdate(GripperState state)
        {
            Debug.Log($"SendGripperStateUpdate! {state}");
            var data = new GripperStateMessage
            {
                State = state
            };
            SendMessage(SkillsMessageHandler.TYPE, SkillsMessageHandler.GRIPPER_STATE, data);
        }

        private void SendFoundObjectMessage(string actionId, string label, Vector3 position)
        {
            var data = new TargetObjectDetectionMessage
            {
                ActionId = actionId,
                Label = label,
                Targets = new[]
                {
                    new TargetObject
                    {
                        Threshold = 1f, Position = RobotConversion.Vector3ToRobotPose(position), Normal = RobotConversion.Vector3ToRobotPose(Vector3.up)
                    }
                }
            };
            SendMessage(SkillsMessageHandler.TYPE, SkillsMessageHandler.TARGET_OBJECT_FOUND, data);
        }

        private void SendDetectingObjectsMessage(string actionId, string label, Vector3 position, int numberOptions)
        {
            var targets = new TargetObject[numberOptions];
            targets[0] = new TargetObject
            {
                Threshold = 1f, Position = RobotConversion.Vector3ToRobotPose(position), Normal = RobotConversion.Vector3ToRobotPose(Vector3.up)
            };

            if (numberOptions > 1)
            {
                for (var i = 1; i < numberOptions; ++i)
                {
                    var randomness = Random.insideUnitCircle;
                    var randomPosition = position;
                    randomPosition.x += randomness.x;
                    randomPosition.z += randomness.y;

                    targets[i] = new TargetObject
                    {
                        Threshold = 1f / i, Position = RobotConversion.Vector3ToRobotPose(randomPosition),
                        Normal = RobotConversion.Vector3ToRobotPose(Vector3.up)
                    };
                }
            }

            Debug.Log($"Targets::{targets.Length}");
            var data = new TargetObjectDetectionMessage
            {
                ActionId = actionId,
                Label = label,
                Targets = targets
            };
            SendMessage(SkillsMessageHandler.TYPE, SkillsMessageHandler.TARGET_OBJECT_DETECTING, data);
        }

        private void SendPoseMessage()
        {
            var data = new RobotPoseMessage
            {
                Position = RobotConversion.Vector2ToRobotPose(_robotSim.CurrentPosition),
                Yaw = _robotSim.CurrentYaw * Mathf.Deg2Rad * -1
            };
            SendMessage(SkillsMessageHandler.TYPE, SkillsMessageHandler.ROBOT_POSE, data);
        }

        private void SendEEPoseMessage()
        {
            var data = new EndEffectorPoseMessage
            {
                Position = RobotConversion.Vector3ToRobotPose(_robotArmSim.CurrentPosition),
                Orientation = RobotConversion.QuaternionToRobotPose(_robotArmSim.CurrentRotation),
                PRY = new YawPitchRoll
                {
                    Yaw = _robotArmSim.CurrentRotation.eulerAngles.y,
                    Pitch = _robotArmSim.CurrentRotation.eulerAngles.x,
                    Roll = _robotArmSim.CurrentRotation.eulerAngles.z
                }
            };
            
            SendMessage(SkillsMessageHandler.TYPE, SkillsMessageHandler.EE_POSE, data);
        }
    }
}