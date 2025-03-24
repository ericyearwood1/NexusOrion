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

using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Tests for the OVRVirtualKeyboard's Unity Input Field Text Handler
/// </summary>
[TestFixture]
[Category("OnCall:xrinput_text_entry")]
class OVRVirtualKeyboardUnityInputFieldTextHandlerTest
{

    private const string InitialText = "Initial Text";

    private InputField _inputField;
    private OVRVirtualKeyboardInputFieldTextHandler _textHandler;
    private int _onEndEditCount;
    private string _lastOnEndEdit;
    private object _lastOnTextChanged;
    private int _onTextChangedCount;

    private void OnTextChanged(string arg0)
    {
        _lastOnTextChanged = arg0;
        _onTextChangedCount++;
    }

    private void OnEndEdit(string arg0)
    {
        _lastOnEndEdit = arg0;
        _onEndEditCount++;
    }

    [SetUp]
    public void SetUp()
    {
        var go = new GameObject();

        _inputField = go.AddComponent<InputField>();
        _inputField.onEndEdit.AddListener(OnEndEdit);

        _textHandler = go.AddComponent<OVRVirtualKeyboardInputFieldTextHandler>();
        _textHandler.InputField = _inputField;
        _textHandler.OnTextChanged += OnTextChanged;

        _inputField.lineType = InputField.LineType.SingleLine;
        _inputField.text = InitialText;
        _lastOnTextChanged = string.Empty;
        _onTextChangedCount = 0;
        _lastOnEndEdit = string.Empty;
        _onEndEditCount = 0;
    }

    [TearDown]
    public void OneTimeTearDown()
    {
        GameObject.DestroyImmediate(_inputField.gameObject);
    }

    [Test]
    public void GetTextReturnsInputFieldText()
    {
        Assert.That(_textHandler.Text, Is.EqualTo(_inputField.text));
    }

    [Test]
    public void GetIsFocusedReturnsInputFieldIsFocused()
    {
        // no reasonable way to make an input field focused in test
        Assert.That(_textHandler.IsFocused, Is.EqualTo(_inputField.isFocused));
    }

    [Test]
    public void GetSubmitOnEnterReturnsBasedOnInputFieldLineType([Values] InputField.LineType lineType)
    {
        _inputField.lineType = lineType;
        if (lineType == InputField.LineType.MultiLineNewline)
        {
            Assert.That(_textHandler.SubmitOnEnter, Is.False);
        }
        else
        {
            Assert.That(_textHandler.SubmitOnEnter, Is.True);
        }


    }

    [Test]
    public void SubmitInvokesOnEndEdit()
    {
        _textHandler.Submit();
        Assert.That(_onEndEditCount, Is.EqualTo(1));
        Assert.That(_lastOnEndEdit, Is.EqualTo(InitialText));
    }

    [Test]
    public void SubmitIgnoresNullInputField()
    {
        _textHandler.InputField = null;
        _textHandler.Submit();
        Assert.That(_onEndEditCount, Is.EqualTo(0));
    }

    [Test]
    public void AppendTextAppendsSuppliedText()
    {
        _textHandler.AppendText("append");
        Assert.That(_inputField.text, Is.EqualTo(InitialText + "append"));
    }

    [Test]
    public void AppendTextIgnoresNullInputField()
    {
        _textHandler.InputField = null;
        _textHandler.AppendText("append");
        Assert.That(_lastOnTextChanged, Is.Empty);
    }

    [Test]
    public void ApplyBackspaceRemovesText()
    {
        _textHandler.ApplyBackspace();
        Assert.That(_inputField.text, Is.EqualTo(InitialText.Substring(0, InitialText.Length - 1)));
    }

    [Test]
    public void ApplyBackspaceDoesNotChangeEmptyText()
    {
        Assume.That(_onTextChangedCount, Is.EqualTo(0));
        _inputField.text = string.Empty;
        Assume.That(_onTextChangedCount, Is.EqualTo(1));
        _textHandler.ApplyBackspace();
        Assert.That(_onTextChangedCount, Is.EqualTo(1));
    }

    [Test]
    public void ApplyBackspaceIgnoresNullInputField()
    {
        _textHandler.InputField = null;
        _textHandler.ApplyBackspace();
        Assert.That(_lastOnTextChanged, Is.Empty);
    }

    [Test]
    public void InputFieldTextChangesInvokeOnTextChanged()
    {
        Assume.That(_onTextChangedCount, Is.EqualTo(0));
        const string testString = "Hello world";
        ; _inputField.text = testString;
        Assert.That(_onTextChangedCount, Is.EqualTo(1));
        Assert.That(_lastOnTextChanged, Is.EqualTo(testString));
    }

    [Test]
    public void GetInputFieldReturnsSetInputField()
    {
        Assert.That(_textHandler.InputField, Is.EqualTo(_inputField));
    }

    [Test]
    public void SetInputFieldToNullInvokesOnTextChange()
    {
        Assume.That(_onTextChangedCount, Is.EqualTo(0));
        _textHandler.InputField = null;
        Assert.That(_onTextChangedCount, Is.EqualTo(1));
        Assert.That(_lastOnTextChanged, Is.EqualTo(string.Empty));
    }

    [Test]
    public void GetTextReturnsEmptyTextWithNullInputField()
    {
        _textHandler.InputField = null;
        Assert.That(_textHandler.Text, Is.EqualTo(string.Empty));
    }
}
