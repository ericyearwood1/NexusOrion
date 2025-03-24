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


#if OVRPLUGIN_TESTING
#if OVRPLUGIN_TESTING_XR_INPUT_TEXT_ENTRY
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

//-------------------------------------------------------------------------------------
/// <summary>
/// Tests for the OVRVirtualKeyboardSampleGame class's Writing Prompt system.
/// </summary>
[TestFixture]
[Category("OnCall:xrinput_text_entry")]
public class OVRVirtualKeyboardSampleWPMPromptInternalTest
{
    private static readonly string kTestPrompt = "The quick brown";
    private float time;
    private OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt writingPrompt;
    private LinkedListNode<OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptWord> firstWord;

    [SetUp]
    public void SetUp()
    {
        time = Time.time;
        writingPrompt = new OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt(kTestPrompt);
        firstWord = writingPrompt.CurrentPrompt;
    }

    [Test]
    public void TestPromptInitialization()
    {
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));
    }

    [Test]
    public void TestPromptWithCorrectFirstInput()
    {
        DoKey("T", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
    }

    [Test]
    public void TestPromptWithIncorrectFirstInput()
    {
        DoKey("w", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect);
    }

    [Test]
    public void TestPromptWithIndividuallyTypedFirstWord()
    {
        DoKey("T", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
        DoKey("h", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
        DoKey("e", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
        // Not complete until space
        Assert.That(writingPrompt.IsComplete, Is.False);
    }

    [Test]
    public void TestPromptWithIndividuallyTypedFirstWordWithSpace()
    {
        DoKey("T", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
        DoKey("h", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
        DoKey("e", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
        DoKey(" ", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New);
        // Advance to next word
        Assert.That(writingPrompt.CurrentPrompt, Is.Not.EqualTo(firstWord));
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));

        Assert.That(writingPrompt.Stats.wordsPerMinute, Is.Zero); // Not 5 characters yet
        Assert.That(writingPrompt.Stats.errorPercentage, Is.Zero);

        // whole prompt still incomplete
        Assert.That(writingPrompt.IsComplete, Is.False);
    }

    [Test]
    public void TestPromptWithIndividuallyTypedFirstWordWithIncorrectLastLetter()
    {
        DoKey("T", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
        DoKey("h", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
        DoKey("e", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
        DoKey("z", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect);
    }

    [Test]
    public void TestPromptWithSwipeTypedFirstWord()
    {
        DoSwipe("The");
        // Advance to next word
        Assert.That(writingPrompt.CurrentPrompt, Is.Not.EqualTo(firstWord));
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));
    }

    [Test]
    public void TestPromptWithSwipeTypedIncorrectFirstWord()
    {
        DoSwipe("They");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect));
        // TODO; this should probably be "1", but the "T" is counting as a valid keystroke

        Assert.That(writingPrompt.Stats.wordsPerMinute, Is.Zero); // Not 5 characters yet
        Assert.That(writingPrompt.Stats.errorPercentage, Is.Zero); // Not a correct first word yet
        Assert.That(writingPrompt.Stats.adjustedWordsPerMinute, Is.Zero);
    }

    [Test]
    public void TestPromptWithSwipeTypedIncorrectFirstWordAlternative()
    {
        DoSwipe("Hey");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect));
        Assert.That(writingPrompt.Stats.wordsPerMinute, Is.Zero); // Not 5 characters yet
        Assert.That(writingPrompt.Stats.errorPercentage, Is.Zero); // Not a correct first word yet
        Assert.That(writingPrompt.Stats.adjustedWordsPerMinute, Is.Zero);
    }

    [Test]
    public void TestPromptWithSwipeTypedIncorrectFirstWordAlternativeCorrection()
    {
        DoSwipe("Hey");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect));
        // Delete "Hey "
        for (int i = 0; i < 4; i++)
        {
            writingPrompt.CommitBackspace(time);
        }

        DoSwipe("The");
        Assert.That(writingPrompt.CurrentPrompt, Is.Not.EqualTo(firstWord));
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));

        Assert.That(writingPrompt.Stats.wordsPerMinute, Is.Zero); // Not 5 characters yet
        Assert.That(writingPrompt.Stats.errorPercentage, Is.EqualTo(0.5f)); // Not a correct first word yet
        Assert.That(writingPrompt.Stats.adjustedWordsPerMinute, Is.Zero); // Not 5 characters yet
    }

    [Test]
    public void TestPromptWithFirstIncorrectKeyWithCorrection()
    {
        DoKey("t", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect);
        writingPrompt.CommitBackspace(time);
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));
        DoKey("T", OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial);
    }

    [Test]
    public void TestPromptWithIndividuallyTypedWholeSentence()
    {
        var queue = new Queue<char>(kTestPrompt.ToCharArray());
        while (queue.Count > 0)
        {
            time += 0.5f;
            writingPrompt.CommitText(queue.Dequeue().ToString(), time);
            if (queue.Count > 0)
            {
                Assert.That(writingPrompt.CurrentPrompt.Value.State,
                    Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New).Or
                        .EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial));
            }
        }

        // No more words
        Assert.That(writingPrompt.CurrentPrompt, Is.Null);
        Assert.That(writingPrompt.IsComplete, Is.True);
        Assert.That(writingPrompt.Stats.totalSeconds, Is.EqualTo(7.0f).Within(0.001f));
        Assert.That(writingPrompt.Stats.wordsPerMinute, Is.EqualTo(25).Within(1));
        Assert.That(writingPrompt.Stats.errorPercentage, Is.Zero);
        Assert.That(writingPrompt.Stats.adjustedWordsPerMinute, Is.EqualTo(writingPrompt.Stats.wordsPerMinute).Within(1));
    }

    [Test]
    public void TestPromptWithSwipeTypedWholeSentence()
    {
        DoSwipe("The");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));

        DoSwipe("quick");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));

        DoSwipe("brown");
        // No more words
        Assert.That(writingPrompt.CurrentPrompt, Is.Null);
        Assert.That(writingPrompt.IsComplete, Is.True);

        Assert.That(writingPrompt.Stats.wordsPerMinute, Is.EqualTo(51));
        Assert.That(writingPrompt.Stats.errorPercentage, Is.Zero);
        Assert.That(writingPrompt.Stats.totalSeconds, Is.EqualTo(3.5f).Within(0.001f));
        Assert.That(writingPrompt.Stats.adjustedWordsPerMinute, Is.EqualTo(writingPrompt.Stats.wordsPerMinute).Within(1));
    }

    [Test]
    public void TestPromptWithSwipeTypedWholeSentenceWithWordSuggestionChange()
    {
        DoSwipe("The");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));

        DoSwipe("quit");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect));
        // simulate suggestion
        // Delete "quit "
        for (int i = 0; i < 5; i++)
        {
            writingPrompt.CommitBackspace(time);
        }

        DoSwipe("quick", true);
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));

        DoSwipe("brown");
        // No more words
        Assert.That(writingPrompt.CurrentPrompt, Is.Null);
        Assert.That(writingPrompt.IsComplete, Is.True);

        Assert.That(writingPrompt.Stats.totalSeconds, Is.GreaterThan(1.5f));
        Assert.That(writingPrompt.Stats.totalSeconds, Is.EqualTo(4f).Within(0.001f));
        Assert.That(writingPrompt.Stats.wordsPerMinute, Is.EqualTo(45).Within(1));
        Assert.That(writingPrompt.Stats.errorPercentage, Is.Zero);
        Assert.That(writingPrompt.Stats.adjustedWordsPerMinute, Is.EqualTo(writingPrompt.Stats.wordsPerMinute).Within(1));
    }

    [Test]
    public void TestPromptWithSwipeTypedWholeSentenceWithWordCorrectionChange()
    {
        DoSwipe("The");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));

        DoSwipe("banana");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect));
        // Delete "banana "
        for (int i = 0; i < 7; i++)
        {
            writingPrompt.CommitBackspace(time);
        }

        DoSwipe("quick");
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));

        DoSwipe("brown");
        // No more words
        Assert.That(writingPrompt.CurrentPrompt, Is.Null);
        Assert.That(writingPrompt.IsComplete, Is.True);

        Assert.That(writingPrompt.Stats.totalSeconds, Is.GreaterThan(1.5f));
        Assert.That(writingPrompt.Stats.totalSeconds, Is.EqualTo(5f).Within(0.001f));
        Assert.That(writingPrompt.Stats.wordsPerMinute, Is.EqualTo(36).Within(1));
        Assert.That(writingPrompt.Stats.mistakes, Is.EqualTo(7));
        Assert.That(writingPrompt.Stats.errorPercentage, Is.EqualTo(0.304).Within(0.001f));
        Assert.That(writingPrompt.Stats.adjustedWordsPerMinute, Is.EqualTo(25).Within(1));
    }

    private void DoKey(string key,
        OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState expectedState =
            OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect)
    {
        time += 0.5f;
        writingPrompt.CommitText(key, time);
        Assert.That(writingPrompt.CurrentPrompt.Value.State,
            Is.EqualTo(expectedState));
    }

    private void DoSwipe(string word, bool isSuggestion = false)
    {
        if (!isSuggestion)
        {
            time += 0.5f;
            writingPrompt.CommitText(word[0].ToString(), time);
            if (writingPrompt.CurrentPrompt.Value.Word[0] == word[0])
            {
                Assert.That(writingPrompt.CurrentPrompt.Value.State,
                    Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Partial));
            }
            else
            {
                Assert.That(writingPrompt.CurrentPrompt.Value.State,
                    Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.Incorrect));
            }

            time += 0.5f;
            writingPrompt.CommitBackspace(time);
            Assert.That(writingPrompt.CurrentPrompt.Value.State,
                Is.EqualTo(OVRVirtualKeyboardSampleWPMPromptInternal.WritingPrompt.PromptState.New));
        }

        time += 0.5f;
        writingPrompt.CommitText(word + " ", time);
    }
}

#endif
#endif // OVRPLUGIN_TESTING
