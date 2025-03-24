using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "MockListData", menuName = "ARDS/MockListData", order = 0)]
    public class MockListData : ScriptableObject
    {
        [SerializeField] private List<ListItemViewModel> _listItemViewModels;
        public List<ListItemViewModel> ListItemViewModels => _listItemViewModels;
    }
}
