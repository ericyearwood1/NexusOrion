using System;
using System.Linq;
using Robot.Runtime.Data;
using Robot.Runtime.Data.Robot;
using Robot.Runtime.Messages.Server.Skills;
using Robot.Runtime.Utils;
using SiroComms.Runtime.Messages.Server;
using SiroComms.Runtime.Services;
using UnityEngine;

namespace Robot.Runtime.ServiceHandlers
{
    public class SkillsMessageHandler : IMessageHandler
    {
        public const string TYPE = "skills";
        public const string EE_POSE = "ee_pose";
        public const string ROBOT_POSE = "robot_pose";
        public const string NAVIGATION_HIGHLIGHT = "navigation_highlight";
        public const string TARGET_OBJECT_DETECTING = "detecting_objects";
        public const string TARGET_OBJECT_FOUND = "found_object";
        public const string GRIPPER_STATE = "gripper_state";
        public const string PLACE_LOCATION = "location_detection";

        public Action<GripperState> OnGripperStateReceived;
        public Action<LabelHighlightData> OnNavigationHighlightReceived;
        public Action<TargetObjectHighlight> OnSelectingTargetsReceived;
        public Action<TargetObjectHighlight> OnTargetFound;
        public Action<PlaceLocationData> OnPlaceLocationReceived;
        
        private readonly RobotData _data;

        public SkillsMessageHandler(RobotData data)
        {
            _data = data;
        }

        public void HandleMessage(string messageType, string message)
        {
            switch (messageType)
            {
                case EE_POSE:
                    HandleEndEffectorPose(message);
                    break;
                case ROBOT_POSE:
                    HandleRobotPose(message);
                    break;
                case NAVIGATION_HIGHLIGHT:
                    HandleNavigationHighlight(message);
                    break;
                case TARGET_OBJECT_DETECTING:
                    HandleDetectingObjects(message);
                    break;
                case TARGET_OBJECT_FOUND:
                    HandleObjectFound(message);
                    break;
                case GRIPPER_STATE:
                    HandleGripperState(message);
                    break;
                case PLACE_LOCATION:
                    HandlePlaceLocationMessage(message);
                    break;
            }
        }

        private void HandlePlaceLocationMessage(string message)
        {
            var placeMessage = ServerMessage<PlaceLocationMessage>.Deserialize(message).Data;
            var highlightData = new PlaceLocationData
            {
                Position = RobotConversion.RobotPoseToVector3(placeMessage.Position),
                ActionId = placeMessage.ActionId,
                TargetOrientation = RobotConversion.RobotPoseToQuaternion(placeMessage.TargetOrientation),
                TargetEuler = RobotConversion.RightHandToLeftHand(placeMessage.TargetEuler)
            };
            highlightData.TargetEuler.x *= -1;
            highlightData.TargetEuler.z *= -1;
            Debug.Log($"Target Euler before conversion: {placeMessage.TargetEuler}\n" +
                      $"Target Euler after conversion: {highlightData.TargetEuler}");
            OnPlaceLocationReceived?.Invoke(highlightData);
        }

        private void HandleGripperState(string message)
        {
            var gripperStateMessage = ServerMessage<GripperStateMessage>.Deserialize(message).Data;
            _data.GripperState = gripperStateMessage.State;
            OnGripperStateReceived?.Invoke(_data.GripperState);
        }

        /**
         * Sent when the robot is determining which object to pick up
         * List of 1-Many objects, with a threshold value to indicate likelihood of being chosen
         */
        private void HandleDetectingObjects(string message)
        {
            var objectDetectionMessage = ServerMessage<TargetObjectDetectionMessage>.Deserialize(message).Data;
            var targets = objectDetectionMessage.Targets.OrderByDescending(x => x.Threshold).ToList();
            var priorityIndex = 1;
            foreach (var target in targets)
            {
                var highlightData = new TargetObjectHighlight
                {
                    Position = RobotConversion.RobotPoseToVector3(target.Position),
                    Normal = RobotConversion.RobotPoseToVector3(target.Normal),
                    Label = objectDetectionMessage.Label,
                    Threshold = target.Threshold,
                    OrderIndex = priorityIndex
                };
                priorityIndex++;
                OnSelectingTargetsReceived?.Invoke(highlightData);
            }
        }
        
        /**
         * Sent when the robot has decided which object to pick up. Should only contain one object
         */
        private void HandleObjectFound(string message)
        {
            TargetObjectDetectionMessage objectDetectionMessage;
            try
            {
                objectDetectionMessage = ServerMessage<TargetObjectDetectionMessage>.Deserialize(message).Data;
            }
            catch (Exception e)
            {
                Debug.LogError($"Error deserializing object found message: {e.Message}");
                return;
            }

            switch (objectDetectionMessage.Targets.Length)
            {
                case > 1:
                    Debug.LogError("Handle object found message has multiple targets. Default to selecting the first");
                    break;
                case 0:
                    Debug.LogError("Handle object found message has no targets. Returning");
                    return;
            }

            var target = objectDetectionMessage.Targets[0];
            var highlightData = new TargetObjectHighlight
            {
                Position = RobotConversion.RobotPoseToVector3(target.Position),
                Normal = RobotConversion.RobotPoseToVector3(target.Normal),
                Label = objectDetectionMessage.Label,
                Threshold = target.Threshold,
                OrderIndex = 1
            };
            OnTargetFound?.Invoke(highlightData);
        }

        private void HandleNavigationHighlight(string message)
        {
            var navigationHighlightMessage = ServerMessage<NavigationHighlightMessage>.Deserialize(message).Data;
            var highlightData = new LabelHighlightData
            {
                Position = RobotConversion.RobotPoseToVector3(navigationHighlightMessage.Position),
                Label = navigationHighlightMessage.Target
            };
            OnNavigationHighlightReceived?.Invoke(highlightData);
        }

        private void HandleEndEffectorPose(string message)
        {
            var eePoseMessage = ServerMessage<EndEffectorPoseMessage>.Deserialize(message).Data;
            _data.EEPosition = RobotConversion.RobotPoseToVector3(eePoseMessage.Position);
            _data.EEOrientation = RobotConversion.RobotPoseToQuaternion(eePoseMessage.Orientation);
            _data.EEYPR = RobotConversion.RightHandToLeftHand(eePoseMessage.PRY);
        }

        private void HandleRobotPose(string message)
        {
            var robotPoseMessage = ServerMessage<RobotPoseMessage>.Deserialize(message).Data;
            _data.Position = RobotConversion.RobotPoseToVector3(robotPoseMessage.Position);
            _data.Rotation.y = Mathf.Rad2Deg * robotPoseMessage.Yaw * -1;
        }
    }
}