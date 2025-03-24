using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using OSIG.Tools.StateMachines;

namespace ProtoKit.UI
{
    public static class PKUIUtils
    {
        // Old State names coming from DesignSuite plugin
        public const string DEFAULT_STATE_NAME = "Default";
        public const string NORMAL_STATE_NAME = "Normal";
        public const string HOVER_STATE_NAME = "Hover";
        public const string PRESS_STATE_NAME = "Press";
        public const string CLICK_STATE_NAME = "Click";
        public const string ACTIVE_STATE_NAME = "Active";
        public const string DISABLE_STATE_NAME = "Disable";

        // New State names for PKUIToggleStateVisuals component

        // PRESSED_ = while pressing state
        // SELECTED_ = pressed state
        public const string DEFAULT_NORMAL_OFF = "Normal_Off";
        public const string DEFAULT_HOVERED_OFF = "Hovered_Off";
        public const string DEFAULT_PRESSED_OFF = "Pressed_Off";
        public const string DEFAULT_SELECTED_OFF = "Selected_Off";

        public const string DEFAULT_NORMAL_ON = "Normal_On";
        public const string DEFAULT_HOVERED_ON = "Hovered_On";
        public const string DEFAULT_PRESSED_ON = "Pressed_On";
        public const string DEFAULT_SELECTED_ON = "Selected_On";

        public const string DEFAULT_GAZE_OFF = "Gaze";
        public const string DEFAULT_GAZE_ON = "Selected + Gaze";

        // New State names for PKUIButtonStateVisuals component
        public const string DEFAULT_NORMAL = "Normal";
        public const string DEFAULT_HOVERED = "Hovered";
        public const string DEFAULT_PRESSED = "Pressed";
        public const string DEFAULT_SELECTED = "Selected";

        public const string ON_STATE_NAME = "On";
        public const string OFF_STATE_NAME = "Off";

        public const string MAIN_VARIANT_NAME = "mainvariant";
        public const string STATES_NAME = "States";
        public const string STATE_NAME = "State";

        public static T FindComponentOnClosestAncestor<T>(Transform trans)
        {
            Transform parentTrans = trans.parent;
            T component = default;
            while (parentTrans != null && component == null)
            {
                component = parentTrans.GetComponent<T>();
                parentTrans = parentTrans.parent;
            }
            return component;
        }

        public static void TryTransitioningToAnyState(StateMachine machine, List<string> stateNames)
        {
            // This functions takes a list of states
            // and try to switch to the first state found
            // it will try all the states one by one as soon as
            // any of these states are found, state-machine  switches
            // to that state.
            // This function is mainly used when we are not sure about
            // the name of the state example: active state of the button
            // can be named as "active", "selected" or "selected_on"

            StateRef stateRef = new StateRef();
            bool stateFound = false;
            for (int i = 0; i < stateNames.Count; i++)
            {
                stateFound = machine.TryFindStateRef(stateNames[i], out stateRef);
                if (stateFound)
                {
                    machine.TransitionToState(stateNames[i]);
                    return;
                }
            }
        }

        public static string FindStateNameBasedOnName(string nodeName) {

            // If "=" exists more than 1 times in nodeName
            // then return nodeName.
            // This is used for names that have stateNames and styling names in them
            // eg: "Size=small, Style= Primary, State = Hover"

            string stateName = default;
            if (nodeName.Contains("=")) {
                int occuranceNum = nodeName.Count(x => x == '=');
                if (occuranceNum > 1) {
                    return nodeName.Replace(" ", "");
                }
                else {
                    stateName = nodeName.Split('=')[1];
                }
                
            }
            else {
                stateName = nodeName;
            }

            stateName = stateName.ToLower().Replace("\"", "");

            if (stateName.Contains("-")) {
                stateName = stateName.Split('-')[0];
            }

            // Using Switch Case, it didn't allow checking
            // as case DEFAULT_STATE_NAME.ToLower():

            if (stateName == DEFAULT_STATE_NAME.ToLower() ||
                stateName == NORMAL_STATE_NAME.ToLower() ||
                stateName == DEFAULT_NORMAL_OFF.ToLower() ||
                stateName == OFF_STATE_NAME.ToLower()) {
                stateName = DEFAULT_NORMAL_OFF;
            }
            else if (stateName == ACTIVE_STATE_NAME.ToLower() ||
                     stateName == DEFAULT_NORMAL_ON.ToLower() ||
                     stateName == ON_STATE_NAME.ToLower()) {
                stateName = DEFAULT_NORMAL_ON;
            }
            else if (stateName == HOVER_STATE_NAME.ToLower() ||
                     stateName == DEFAULT_HOVERED_OFF.ToLower() ||
                     stateName == DEFAULT_GAZE_OFF.ToLower()) {
                stateName = DEFAULT_HOVERED_OFF;
            }
            else if (stateName == DEFAULT_HOVERED_ON.ToLower() ||
                     stateName == DEFAULT_GAZE_ON.ToLower()) {
                stateName = DEFAULT_HOVERED_ON;
            }
            else if (stateName == PRESS_STATE_NAME.ToLower() ||
                     stateName == CLICK_STATE_NAME.ToLower() ||
                     stateName == DEFAULT_PRESSED_OFF.ToLower()) {
                stateName = DEFAULT_PRESSED_OFF;
            }
            else if (stateName == DEFAULT_PRESSED_ON.ToLower()) {
                stateName = DEFAULT_PRESSED_ON;
            }
            else if (stateName == DEFAULT_SELECTED_OFF.ToLower()) {
                stateName = DEFAULT_SELECTED_OFF;
            }
            else if (stateName == DEFAULT_SELECTED_ON.ToLower()) {
                stateName = DEFAULT_SELECTED_ON;
            }
            return stateName;
        }


        public static List<string> MapEightStatesBasedOnAvailableStates(List<string> stateNames) {

            // This function returns the name of the states that should be assigned to 8-states of StateVisual component
            // depending on which state exists
            // example : 

            string normal_off_stateName = stateNames[0];
            string normal_on_stateName = stateNames[1];
            string hovered_off_stateName = stateNames[2];
            string hovered_on_stateName = stateNames[3];
            string pressed_off_stateName = stateNames[4];
            string pressed_on_stateName = stateNames[5];
            string selected_off_stateName = stateNames[6];
            string selected_on_stateName = stateNames[7];


            // Null coallescing for returning first non-null value
            // https://learn.microsoft.com/en-us/dotnet/csharp/language-reference/operators/null-coalescing-operator
            string normal_off_assigned_stateName = normal_off_stateName;
            string normal_on_assigned_stateName = normal_on_stateName ?? normal_off_assigned_stateName;

            string hovered_off_assigned_stateName = hovered_off_stateName ?? normal_off_assigned_stateName;
            string hovered_on_assigned_stateName = hovered_on_stateName ?? hovered_off_assigned_stateName ?? normal_on_assigned_stateName;

            string pressed_off_assigned_stateName = pressed_off_stateName ?? hovered_off_assigned_stateName;
            string pressed_on_assigned_stateName = pressed_on_stateName ?? pressed_off_stateName ?? hovered_on_assigned_stateName;

            string selected_off_assigned_stateName = selected_off_stateName ?? pressed_off_assigned_stateName;
            // above line is equivalent of :
            // string selected_off_assigned_state = selected_off_stateName ?? pressed_off_stateName ?? hovered_off_stateName ?? normal_off_stateName;

            string selected_on_assigned_stateName = selected_on_stateName ?? selected_off_assigned_stateName ?? pressed_on_assigned_stateName;

            List<string> assignedStates = new List<string>{
                    normal_off_assigned_stateName, normal_on_assigned_stateName,
                    hovered_off_assigned_stateName, hovered_on_assigned_stateName,
                    pressed_off_assigned_stateName, pressed_on_assigned_stateName,
                    selected_off_assigned_stateName, selected_on_assigned_stateName };

            return assignedStates;
        }
    }
}
