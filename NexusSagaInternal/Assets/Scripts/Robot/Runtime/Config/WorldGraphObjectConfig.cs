using System.Collections.Generic;
using UnityEngine;

namespace Robot.Runtime.Config
{
    [CreateAssetMenu(menuName = "Nexus/World Graph Object Config", fileName = "World Graph Object Config")]
    public class WorldGraphObjectConfig : ScriptableObject
    {
        public Sprite UnknownObject;
        public Sprite UnknownFurniture;
        public bool IsUseObjectIdForFurnitureLabel;
        public List<string> FlatDisplayList = new List<string> { "table","console","counter","island","cabinet"};
        public List<WorldGraphAssetMapping> Mappings = new List<WorldGraphAssetMapping>
        {
            new() { Id = "party_hat" },
            new() { Id = "donut" },
            new() { Id = "cup" },
            new() { Id = "can" },
            new() { Id = "bottle" },
            new() { Id = "dining_table"},
            new() { Id = "kitchen_island"},
            new() { Id = "kitchen_counter"},
            new() { Id = "sofa" },
            new() { Id = "wooden_console"},
            new() { Id = "wooden_dresser" },
            new() { Id = "coffee_table"},
            new() { Id = "office_chair" },
            new() { Id = "cabinet"},
            new() { Id = "drawer" },
            new() { Id = "trash_can" },
            new() { Id = "shelf" },
            new() { Id = "chair" },
            new() { Id = "hamper" },
            new() { Id = "pillow" },
            new() { Id = "pineapple"},
            new() { Id = "refrigerator" },
            new() { Id = "bed" }
        };
    }
}