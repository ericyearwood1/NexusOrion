/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if OVR_INTERNAL_CODE
using UnityEngine;
using UnityEngine.UI;
using System.Text;

public class OVRBodyTest : MonoBehaviour
{
    public class BoolMonitor
    {
        public delegate bool BoolGenerator();

        private string _name;
        private BoolGenerator _generator;
        private bool _prevValue;
        private bool _currentValue;
        private bool _currentValueRecentlyChanged;
        private float _displayTimeout;
        private float _displayTimer;

        public BoolMonitor(string name, BoolGenerator generator, float displayTimeout = 0.5f)
        {
            _name = name;
            _generator = generator;
            _displayTimeout = displayTimeout;
        }

        public void Update()
        {
            _prevValue = _currentValue;
            _currentValue = _generator();

            if (_currentValue != _prevValue)
            {
                _currentValueRecentlyChanged = true;
                _displayTimer = _displayTimeout;
            }

            if (_displayTimer > 0.0f)
            {
                _displayTimer -= Time.deltaTime;

                if (_displayTimer <= 0.0f)
                {
                    _currentValueRecentlyChanged = false;
                    _displayTimer = 0.0f;
                }
            }
        }

        public void AppendToStringBuilder(StringBuilder sb)
        {
            sb.Append(_name);

            if (_currentValue && _currentValueRecentlyChanged)
                sb.AppendLine(": *True*");
            else if (_currentValue)
                sb.AppendLine(":  True ");
            else if (!_currentValue && _currentValueRecentlyChanged)
                sb.AppendLine(": *False*");
            else if (!_currentValue)
                sb.AppendLine(":  False ");
        }
    }

    public Text uiText;
    private BoolMonitor[] _monitors;
    private StringBuilder _stringBuilder = new StringBuilder();

    private OVRPlugin.BodyState _bodyState;
    private OVRPlugin.Skeleton2 _skeleton;

    private bool _getSkeletonResult;
    private OVRFaceExpressions _faceExpressions;
    private OVREyeGaze _eyeGaze;

    private void Start()
    {
        if (uiText != null)
        {
            uiText.supportRichText = false;
        }

        _monitors = new[]
        {
            new BoolMonitor("One", () => OVRInput.Get(OVRInput.Button.One)),
            new BoolMonitor("Two", () => OVRInput.Get(OVRInput.Button.Two)),
            new BoolMonitor("Three", () => OVRInput.Get(OVRInput.Button.Three)),
            new BoolMonitor("Four", () => OVRInput.Get(OVRInput.Button.Four)),
        };

        _getSkeletonResult = OVRPlugin.GetSkeleton2(OVRPlugin.SkeletonType.Body, ref _skeleton);

        _faceExpressions = FindAnyObjectByType<OVRFaceExpressions>();
        _eyeGaze = FindAnyObjectByType<OVREyeGaze>();
    }

    private static string _prevConnected = string.Empty;

    private static readonly BoolMonitor Controllers = new BoolMonitor("Controllers Changed",
        () => OVRInput.GetConnectedControllers().ToString() != _prevConnected);

    private void Update()
    {
        _stringBuilder.Clear();

        var activeController = OVRInput.GetActiveController();

        var activeControllerName = activeController.ToString();
        _stringBuilder.AppendLine($"Active: {activeControllerName}");

        var connectedControllerNames = OVRInput.GetConnectedControllers().ToString();
        _stringBuilder.AppendLine($"Connected: {connectedControllerNames}");

        _stringBuilder.AppendLine($"PrevConnected: {_prevConnected}");

        Controllers.Update();
        Controllers.AppendToStringBuilder(_stringBuilder);
        _prevConnected = connectedControllerNames;

        _stringBuilder.AppendLine($"HandTrackingEnabled: {OVRPlugin.GetHandTrackingEnabled()}");

        _stringBuilder.AppendLine($"{nameof(OVRPlugin.bodyTrackingSupported)} => {OVRPlugin.bodyTrackingSupported}");
        _stringBuilder.AppendLine($"{nameof(OVRPlugin.bodyTrackingEnabled)} => {OVRPlugin.bodyTrackingEnabled}");
        _stringBuilder.AppendLine($"{nameof(OVRPlugin.faceTrackingSupported)} => {OVRPlugin.faceTrackingSupported}");
        _stringBuilder.AppendLine($"{nameof(OVRPlugin.faceTrackingEnabled)} => {OVRPlugin.faceTrackingEnabled}");
        _stringBuilder.AppendLine($"{nameof(OVRPlugin.eyeTrackingSupported)} => {OVRPlugin.eyeTrackingSupported}");
        _stringBuilder.AppendLine($"{nameof(OVRPlugin.eyeTrackingEnabled)} => {OVRPlugin.eyeTrackingEnabled}");

        var result = OVRPlugin.GetBodyState(OVRPlugin.Step.Render, ref _bodyState);
        _stringBuilder.AppendLine($"{nameof(OVRPlugin.GetBodyState)} => {result}");
        _stringBuilder.AppendLine($"BodyState {nameof(_bodyState.Confidence)}: {_bodyState.Confidence}");
        _stringBuilder.AppendLine(
            $"BodyState {nameof(_bodyState.SkeletonChangedCount)}: {_bodyState.SkeletonChangedCount}");
        _stringBuilder.AppendLine($"BodyState {nameof(_bodyState.Time)}: {_bodyState.Time}");

        _stringBuilder.AppendLine($"Skeleton Query Res: {_getSkeletonResult}");
        _stringBuilder.AppendLine($"Skeleton Type: {_skeleton.Type}");
        _stringBuilder.AppendLine($"Skeleton NumBones: {_skeleton.NumBones}");
        _stringBuilder.AppendLine($"Skeleton NumBoneCapsules: {_skeleton.NumBoneCapsules}");

        foreach (var monitor in _monitors)
        {
            monitor.Update();
            monitor.AppendToStringBuilder(_stringBuilder);
        }

        if (uiText != null)
        {
            uiText.text = _stringBuilder.ToString();
        }
    }
}
#endif
