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
#if OVRPLUGIN_TESTING
#if OVR_INTERNAL_TESTING_GUI

using System;
using System.Collections;
using System.Reflection;
using UnityEngine;
using NUnit.Framework;
using UnityEditor;
using UnityEngine.TestTools;

internal class OVREditorUtilsTests : OVRPluginEditModeTest
{
    private const string TestId = "testId";
    private TestWindow Window;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        OVREditorUtils.HoverHelper.Reset();
        OVREditorUtils.TweenHelper.Reset();
        Window = EditorWindow.GetWindow<TestWindow>();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        Window.Close();
        OVREditorUtils.TweenHelper.Reset();
        OVREditorUtils.HoverHelper.Reset();

        yield return base.UnityTearDown();
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_UpdateEditorIsRegistered()
    {
        var field = typeof(EditorApplication).GetField("update", BindingFlags.Static | BindingFlags.Public);
        var multicastDelegate = (Delegate)field?.GetValue(null);
        var delegates = multicastDelegate?.GetInvocationList();
        var updateEditorMethod =
            typeof(OVREditorUtils).GetMethod("UpdateEditor", BindingFlags.Static | BindingFlags.NonPublic);
        var found = false;
        foreach (var del in delegates)
        {
            if (del.Method == updateEditorMethod)
            {
                found = true;
                break;
            }
        }
        Assert.IsTrue(found);

        yield return null;
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_DeltaTimeUpdatesCorrectly()
    {
        var initialUpdateTime = OVREditorUtils.LastUpdateTime;

        yield return null;

        var nextUpdateTime = OVREditorUtils.LastUpdateTime;
        var deltaTime = OVREditorUtils.DeltaTime;
        var computedDeltaTime = (float)(nextUpdateTime - initialUpdateTime);

        Assert.AreNotEqual(initialUpdateTime, nextUpdateTime);
        Assert.AreNotEqual(deltaTime, 0);
        Assert.AreEqual(computedDeltaTime, deltaTime);
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Hover_OnlyIfRepaint()
    {
        var rect = new Rect(10, 10, 100, 100);

        foreach (var en in Enum.GetValues(typeof(EventType)))
        {
            var e = new Event { type = (EventType)en, mousePosition = new Vector2(50, 50) };
            Assert.AreEqual(OVREditorUtils.HoverHelper.IsHover(en.ToString(), e, rect),(EventType)en == EventType.Repaint);
        }

        yield return null;
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Hover_ReturnsTrueWhenMouseInside()
    {
        var rect = new Rect(10, 10, 100, 100);
        var e = new Event { type = EventType.Repaint, mousePosition = new Vector2(50, 50) };

        Assert.IsTrue(OVREditorUtils.HoverHelper.IsHover(TestId, e, rect));

        yield return null;
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Hover_ReturnsFalseWhenMouseOutside()
    {
        var rect = new Rect(10, 10, 100, 100);
        var e = new Event { type = EventType.Repaint, mousePosition = new Vector2(200, 200) };

        Assert.IsFalse(OVREditorUtils.HoverHelper.IsHover(TestId, e, rect));

        yield return null;
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Hover_InvalidParametersIsGetter()
    {
        var e = new Event { type = EventType.Repaint, mousePosition = new Vector2(50, 50) };
        var rect = new Rect(10, 10, 100, 100);

        Assert.IsFalse(OVREditorUtils.HoverHelper.IsHover(TestId, e, null));
        Assert.IsFalse(OVREditorUtils.HoverHelper.IsHover(TestId, null, null));
        Assert.IsFalse(OVREditorUtils.HoverHelper.IsHover(TestId, null, rect));

        Assert.IsTrue(OVREditorUtils.HoverHelper.IsHover(TestId, e, rect));

        Assert.IsTrue(OVREditorUtils.HoverHelper.IsHover(TestId, e, null));
        Assert.IsTrue(OVREditorUtils.HoverHelper.IsHover(TestId, null, null));
        Assert.IsTrue(OVREditorUtils.HoverHelper.IsHover(TestId, null, rect));

        yield return null;
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Hover_Button_Over()
    {
        var hover = false;
        Window.TestGUI = () =>
        {
            EditorGUILayout.BeginHorizontal();
            Event.current.mousePosition = new Vector2(10, 10);
            OVREditorUtils.HoverHelper.Button(TestId, new GUIContent("Button"), EditorStyles.miniButton,
                out hover);
            EditorGUILayout.EndHorizontal();
        };

        yield return null;

        Assert.IsTrue(hover);
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Hover_Button_Out()
    {
        var hover = false;
        Window.TestGUI = () =>
        {
            EditorGUILayout.BeginHorizontal();
            Event.current.mousePosition = new Vector2(150, 150);
            OVREditorUtils.HoverHelper.Button(TestId, new GUIContent("Button"), EditorStyles.miniButton,
                out hover);
            EditorGUILayout.EndHorizontal();
        };

        yield return null;

        Assert.IsFalse(hover);
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Hover_ButtonWithRect_Over()
    {
        var hover = false;
        Window.TestGUI = () =>
        {
            var rect = GUILayoutUtility.GetRect(100, 100);
            Event.current.mousePosition = new Vector2(50, 50);
            OVREditorUtils.HoverHelper.Button(TestId, rect, new GUIContent("Button"), EditorStyles.miniButton,
                out hover);
        };

        yield return null;

        Assert.IsTrue(hover);
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Hover_ButtonWithRect_Out()
    {
        var hover = false;
        Window.TestGUI = () =>
        {
            var rect = GUILayoutUtility.GetRect(100, 100);
            Event.current.mousePosition = new Vector2(150, 150);
            OVREditorUtils.HoverHelper.Button(TestId, rect, new GUIContent("Button"), EditorStyles.miniButton,
                out hover);
        };

        yield return null;

        Assert.IsFalse(hover);
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Tween_Smooth()
    {
        const float target = 100.0f;
        const float start = 0.0f;
        var smoothedValue = start;
        var previousValue = start;
        var completed = false;
        yield return null;
        do
        {
            Assert.IsFalse(completed);
            smoothedValue = OVREditorUtils.TweenHelper.Smooth(TestId, target, out completed, smoothedValue);
            Assert.That(smoothedValue, Is.GreaterThan(previousValue));
            previousValue = smoothedValue;
            yield return null;
        } while (Math.Abs(smoothedValue - target) > Mathf.Epsilon);
        Assert.AreEqual(smoothedValue, target);
        Assert.IsTrue(completed);
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator EditorUtils_Tween_GUISmooth()
    {
        const float target = 100.0f;
        const float start = 0.0f;
        var smoothedValue = start;
        var paintedValue = start;
        var numberOfRepaint = 0;
        var expectedNumberOfRepaint = 0;
        Window.TestGUI = () =>
        {
            smoothedValue = OVREditorUtils.TweenHelper.GUISmooth(TestId, target, smoothedValue, ifNotCompletedDelegate: () => Window.Repaint());
            if (Event.current.type == EventType.Repaint)
            {
                Assert.AreNotEqual(paintedValue, smoothedValue);
                numberOfRepaint++;
                paintedValue = smoothedValue;
            }
        };

        while (Math.Abs(smoothedValue - target) > Mathf.Epsilon)
        {
            expectedNumberOfRepaint++;
            yield return null;
        }

        Assert.AreEqual(numberOfRepaint, expectedNumberOfRepaint);
    }

    public class TestWindow : EditorWindow
    {
        public Action TestGUI { get; set; }

        private void OnGUI()
        {
            TestGUI?.Invoke();
        }
    }
}

#endif // OVR_INTERNAL_TESTING_GUI
#endif // OVRPLUGIN_TESTING
#endif // OVR_INTERNAL_CODE
