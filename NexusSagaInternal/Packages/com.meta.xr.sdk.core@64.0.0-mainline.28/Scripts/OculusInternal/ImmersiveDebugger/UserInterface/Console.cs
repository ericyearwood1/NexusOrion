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

using Meta.XR.ImmersiveDebugger.UserInterface.Generic;
using Meta.XR.ImmersiveDebugger.Utils;
using System;
using System.Collections;
using System.Collections.Generic;
using Meta.XR.ImmersiveDebugger.DebugData;
using UnityEngine;

namespace Meta.XR.ImmersiveDebugger.UserInterface
{
    public class Console : DebugPanel
    {
        private const int NumberOfLines = 12;

        internal bool Dirty { get; set; }

        private ScrollView _scrollView;
        private Flex LogFlex => _scrollView.Flex;

        private Flex _flex;
        private Flex _buttonsAnchor;

        private List<SeverityEntry> _severities = new List<SeverityEntry>();
        private Dictionary<LogType, SeverityEntry> _severitiesPerType = new Dictionary<LogType, SeverityEntry>();

        private SeverityEntry GetSeverity(LogType logType)
        {
            return _severitiesPerType.TryGetValue(logType, out var severity) ? severity : null;
        }

        private readonly List<ConsoleLine> _lines = new List<ConsoleLine>();
        private readonly List<LogEntry> _entries = new List<LogEntry>();

        protected override void Setup(Controller owner)
        {
            base.Setup(owner);

            _flex = Append<Flex>("main");
            _flex.LayoutStyle = Style.Load<LayoutStyle>("ConsoleFlex");

            // List of Panel Buttons
            _buttonsAnchor = _flex.Append<Flex>("buttons");
            _buttonsAnchor.LayoutStyle = Style.Load<LayoutStyle>("ConsoleButtons");

            RegisterControl("Clear", Resources.Load<Texture2D>("Textures/bin_icon"), Style.Load<ImageStyle>("BinIcon"), Clear);

            var errorSeverity = new SeverityEntry(this, "Error", Resources.Load<Texture2D>("Textures/error_icon"), Style.Load<ImageStyle>("ErrorIcon"), Style.Load<ImageStyle>("PillError"));
            var warningSeverity = new SeverityEntry(this, "Warning", Resources.Load<Texture2D>("Textures/warning_icon"), Style.Load<ImageStyle>("WarningIcon"), Style.Load<ImageStyle>("PillWarning"));
            var infoSeverity = new SeverityEntry(this, "Log", Resources.Load<Texture2D>("Textures/notice_icon"), Style.Load<ImageStyle>("NoticeIcon"), Style.Load<ImageStyle>("PillInfo"));
            _severities.Add(infoSeverity);
            _severities.Add(warningSeverity);
            _severities.Add(errorSeverity);
            _severitiesPerType.Add(LogType.Assert, errorSeverity);
            _severitiesPerType.Add(LogType.Error, errorSeverity);
            _severitiesPerType.Add(LogType.Exception, errorSeverity);
            _severitiesPerType.Add(LogType.Warning, warningSeverity);
            _severitiesPerType.Add(LogType.Log, infoSeverity);

            var runtimeSettings = RuntimeSettings.Instance;
            errorSeverity.ShouldShow = runtimeSettings.ShowErrorLog;
            warningSeverity.ShouldShow = runtimeSettings.ShowWarningLog;
            infoSeverity.ShouldShow = runtimeSettings.ShowInfoLog;

            // List for Log
            _scrollView = Append<ScrollView>("logs");
            _scrollView.LayoutStyle = Style.Load<LayoutStyle>("LogsScrollView");
            LogFlex.LayoutStyle = Style.Load<LayoutStyle>("ConsoleLogs");

            _flex.RefreshLayout();
        }

        protected void OnEnable()
        {
            ConsoleLogsCache.OnLogReceived += EnqueueLogEntry;
            ConsoleLogsCache.ConsumeStartupLogs(EnqueueLogEntry);
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            ConsoleLogsCache.OnLogReceived -= EnqueueLogEntry;
        }

        public Label RegisterCount()
        {
            var label = _buttonsAnchor.Append<Label>("");
            label.LayoutStyle = Style.Load<LayoutStyle>("ConsoleButtonCount");
            label.TextStyle = Style.Load<TextStyle>("ConsoleButtonCount");
            _buttonsAnchor.RefreshLayout();
            return label;
        }

        public Toggle RegisterControl(string buttonName, Texture2D icon, ImageStyle style, Action callback)
        {
            if (buttonName == null) throw new ArgumentNullException(nameof(buttonName));
            if (icon == null) throw new ArgumentNullException(nameof(icon));
            if (callback == null) throw new ArgumentNullException(nameof(callback));

            var toggle = _buttonsAnchor.Append<Toggle>(buttonName);
            toggle.LayoutStyle = Style.Load<LayoutStyle>("ConsoleButton");
            toggle.Icon = icon;
            toggle.IconStyle = style ? style : Style.Default<ImageStyle>();
            toggle.Callback = callback;
            _buttonsAnchor.RefreshLayout();
            return toggle;
        }

        private void EnqueueLogEntry(string logString, string stackTrace, LogType type)
        {
            var severity = GetSeverity(type);
            if (severity == null)
            {
                return;
            }

            var logEntry = new LogEntry(logString, stackTrace, severity);
            _entries.Add(logEntry);
            severity.Count++;

            Dirty = true;
        }

        private void Update()
        {
            if (Dirty)
            {
                RefreshLines();
            }
        }

        private ConsoleLine AddLine()
        {
            var line = LogFlex.Append<ConsoleLine>("log");
            line.LayoutStyle = Style.Load<LayoutStyle>("ConsoleLine");
            _lines.Add(line);
            return line;
        }

        private void Clear()
        {
            _entries.Clear();
            foreach (var severity in _severities)
            {
                severity.Reset();
            }
            Dirty = true;
        }

        private void RefreshLines()
        {
            var previousScroll = _scrollView.ScrollRect.verticalNormalizedPosition;
            // Populates lines with entries
            var index = 0;
            var entriesCount = _entries.Count;
            for (var i = entriesCount - 1; i >= 0; i--)
            {
                var entry = _entries[i];
                if (!entry.Severity.ShouldShow)
                {
                    continue;
                }

                var line = index < _lines.Count ? _lines[index] : AddLine();
                line.Entry = entry;

                index++;
            }

            // Clear remaining lines
            for (var i= _lines.Count - 1; i >= index; i--)
            {
                var line = _lines[index];
                LogFlex.Remove(line, true);
                _lines.RemoveAt(index);
            }

            // Refresh and undirty
            LogFlex.RefreshLayout();
            _scrollView.ScrollRect.verticalNormalizedPosition = previousScroll;
            Dirty = false;
        }
    }
}

#endif
