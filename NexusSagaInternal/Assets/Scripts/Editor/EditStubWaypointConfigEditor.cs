using System.Linq;
using Robot.Runtime.Config;
using Robot.Runtime.Data;
using Robot.Runtime.Data.WorldGraph;
using UnityEditor;
using UnityEngine;
using UnityEngine.ProBuilder;

namespace Robot
{
    public class EditStubWaypointConfigEditor : EditorWindow
    {
        private WaypointConfig _stubWaypointConfig;
        private WorldGraphConfig _stubWorldGraphConfig;
        private GameObject _waypointTarget;
        private MeshFilter _nodeTarget;
        private GameObject _robotHome;
        private GameObject _robot;
        private GameObject _receptacle;
        private GameObject _item;

        [MenuItem("Window/Nexus/Edit Stub Configs", false, 3010)]
        private static void ShowWindow()
        {
            var window = GetWindow<EditStubWaypointConfigEditor>();

            window.titleContent = new GUIContent("Edit Stub Config");
        }

        private void OnGUI()
        {
            EditorGUILayout.LabelField("Configure Stub Waypoints");
            EditorGUILayout.Space();
            _robotHome =
                EditorGUILayout.ObjectField("Robot Home", _robotHome, typeof(GameObject), true) as
                    GameObject;

            _robot =
                EditorGUILayout.ObjectField("Robot", _robot, typeof(GameObject), true) as
                    GameObject;

            EditorGUILayout.Space();
            _stubWaypointConfig =
                EditorGUILayout.ObjectField("Waypoint Config", _stubWaypointConfig, typeof(WaypointConfig), false) as
                    WaypointConfig;

            _stubWorldGraphConfig =
                EditorGUILayout.ObjectField("World Graph Config", _stubWorldGraphConfig, typeof(WorldGraphConfig),
                        false) as
                    WorldGraphConfig;

            ConfigureWaypoints();
            ConfigureWorldGraphNodes();
            ConfigureWorldGraphEdges();

            if (GUILayout.Button("Force Save Config"))
            {
                SaveConfig();
            }
        }

        private void ConfigureWorldGraphNodes()
        {
            EditorGUILayout.LabelField("Add Node to World Graph");
            _nodeTarget =
                EditorGUILayout.ObjectField("Node", _nodeTarget, typeof(MeshFilter), true) as
                    MeshFilter;
            EditorGUI.BeginDisabledGroup(_stubWorldGraphConfig == null || _nodeTarget == null);
            if (GUILayout.Button("Add Object"))
            {
                AddNewNode(false);
            }
            if (GUILayout.Button("Add Furniture"))
            {
                AddNewNode(true);
            }

            EditorGUILayout.LabelField("Add World Graph Edge");

            EditorGUI.EndDisabledGroup();
        }

        private void AddNewNode(bool isFurniture)
        {
            if (_nodeTarget != null && _stubWorldGraphConfig != null)
            {
                var name = _nodeTarget.name;
                var robotHomePosition = _robotHome.transform.position;
                var position = _robotHome.transform.InverseTransformPoint(_nodeTarget.transform.position);
                var renderer = _nodeTarget.GetComponent<MeshRenderer>();
                var bounds = renderer.bounds;
                var localPosition = bounds.center - robotHomePosition;
                var objectPosition = new Vector3(localPosition.x, localPosition.y, localPosition.z);
                var existing = _stubWorldGraphConfig.Graph.Nodes.FirstOrDefault(a => a.Id == name);
                var rotation = renderer.transform.localEulerAngles;
                Debug.Log($"object position:: {position} | {localPosition} | {bounds.center}");
                if (existing == null)
                {
                    _stubWorldGraphConfig.Graph.Nodes.Add(new WorldGraphNode
                    {
                        Id = name,
                        ObjectName = GetObjectNameFromId(name),
                        Category = isFurniture ? "furniture" : _nodeTarget.name,
                        Room = "kitchen",
                        Extents = bounds.extents,
                        Position = _nodeTarget.transform.localPosition,
                        Yaw = rotation.y
                    });
                }
                else
                {
                    existing.Position = objectPosition;
                    existing.Yaw = rotation.y;
                    existing.Extents = bounds.extents;
                }

                SaveConfig();
            }
        }

        private string GetObjectNameFromId(string id)
        {
            var bracketIndex = id.IndexOf("[");
            return bracketIndex < 0 ? id : id.Substring(0, bracketIndex);
        }

        private void ConfigureWorldGraphEdges()
        {
            EditorGUILayout.LabelField("Add Edge to World Graph");
            _receptacle =
                EditorGUILayout.ObjectField("Receptacle", _receptacle, typeof(GameObject), true) as
                    GameObject;
            _item =
                EditorGUILayout.ObjectField("Item", _item, typeof(GameObject), true) as
                    GameObject;
            EditorGUI.BeginDisabledGroup(_stubWorldGraphConfig == null || _receptacle == null || _item == null);
            if (GUILayout.Button("Add Edge"))
            {
                if (_receptacle != null && _item != null && _robot != null && _stubWorldGraphConfig != null)
                {
                    var _receptacleName = _receptacle.name;
                    var _itemName = _item.name;
                    var existing = _stubWorldGraphConfig.Graph.Edges.FirstOrDefault(a =>
                        a.SourceId == _receptacleName && a.TargetId == _itemName);
                    if (existing == null)
                    {
                        _stubWorldGraphConfig.Graph.Edges.Add(new WorldGraphEdge
                        {
                            SourceId = _item.name,
                            TargetId = _receptacle.name,
                            Relationship = NodeRelationship.ON
                        });
                    }

                    SaveConfig();
                }
            }

            EditorGUILayout.LabelField("Add World Graph Edge");

            EditorGUI.EndDisabledGroup();
        }

        private void ConfigureWaypoints()
        {
            EditorGUILayout.LabelField("Configure Waypoints");
            _waypointTarget =
                EditorGUILayout.ObjectField("Waypoint Target", _waypointTarget, typeof(GameObject), true) as
                    GameObject;
            EditorGUI.BeginDisabledGroup(_stubWaypointConfig == null || _waypointTarget == null);
            if (GUILayout.Button("Add Waypoint"))
            {
                if (_waypointTarget != null && _robot != null && _stubWaypointConfig != null)
                {
                    var name = _waypointTarget.name;
                    var robotHomeYaw = _robotHome.transform.eulerAngles.y;
                    var robotRotation = _robot.transform.eulerAngles.y;
                    var waypointYaw = robotRotation - robotHomeYaw;
                    var position = _robot.transform.localPosition;
                    var waypointPosition = new Vector2(position.x, position.z);
                    var existing = _stubWaypointConfig.Waypoints.FirstOrDefault(a => a.Name == name);
                    if (existing == null)
                    {
                        _stubWaypointConfig.Waypoints.Add(new Waypoint
                            { Name = _waypointTarget.name, Position = waypointPosition, Yaw = waypointYaw });
                    }
                    else
                    {
                        existing.Position = waypointPosition;
                        existing.Yaw = waypointYaw;
                    }

                    SaveConfig();
                }
            }

            EditorGUI.EndDisabledGroup();
        }

        private void SaveConfig()
        {
            EditorUtility.SetDirty(_stubWorldGraphConfig);
            EditorUtility.SetDirty(_stubWaypointConfig);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
        }
    }
}