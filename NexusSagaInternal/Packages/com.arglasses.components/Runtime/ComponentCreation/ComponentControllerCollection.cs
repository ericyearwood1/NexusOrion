using System;
using System.Collections.Generic;
using FigmaUnity.FigmaComponents;
using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "ComponentStyleDictionary", menuName = "ARDS/ComponentControllerCollection", order = 1)]
    public class ComponentControllerCollection : ScriptableObject
    {
        public static readonly string ResourcesPath = "ComponentControllerCollection";

        [SerializeField] private ArgButton _aRGButtonControllerPrefab;
        [SerializeField] private ArgAlertDialogue _aRGAlertDialogueontrollerPrefab;
        [SerializeField] private ArgAvatar _argAvatar;
        [SerializeField] private ArgExpButton _argExpButton;
        [SerializeField] private ArgListItem _argListItem;
        [SerializeField] private ArgTooltip _argTooltip;

        public ArgButton InstantiateARGButton(Transform transform)
        {
            return Instantiate(_aRGButtonControllerPrefab, transform);
        }

        public ArgAlertDialogue InstantiateARGAlertDialogue(Transform parent)
        {
            return Instantiate(_aRGAlertDialogueontrollerPrefab, parent);
        }

        public ArgAvatar InstantiateARGAvatar(Transform parent)
        {
            return Instantiate(_argAvatar, parent);
        }

        public ArgExpButton InstantiateARGExpButton(Transform parent)
        {
            return Instantiate(_argExpButton, parent);
        }

        public ArgListItem InstantiateARGListItem(Transform parent)
        {
            return Instantiate(_argListItem, parent);
        }
        
        public ArgTooltip InstantiateARGTooltip(Transform parent)
        {
            return Instantiate(_argTooltip, parent);
        }
    }
}
