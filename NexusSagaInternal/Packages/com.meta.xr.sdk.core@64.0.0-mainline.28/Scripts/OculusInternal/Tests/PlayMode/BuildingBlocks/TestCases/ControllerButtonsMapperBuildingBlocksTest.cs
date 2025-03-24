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
using System.Linq;
using Meta.XR.BuildingBlocks;
using Meta.XR.BuildingBlocks.Editor;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.Events;

#if OVRPLUGIN_TESTING

public class ControllerButtonsMapperBuildingBlocksTest : BuildingBlocksTest
{
    public override string BlockId => BlockDataIds.ControllerButtonsMapper;

    private AnchorTestFixture _testFixture;
    private ControllerButtonsMapper _controllerButtonsMapper;
    private int _onKeyDownCount;
    private int _onKeyUpCount;

    public override IEnumerator Setup(TestContext testContext)
    {
        _testFixture = testContext.SceneManagerTestFixture;
        yield return new WaitUntil((() => OVRManager.OVRManagerinitialized));

        SetupActions();
        yield return null;
    }

    public override IEnumerator TearDown(TestContext testContext)
    {
        _controllerButtonsMapper.ButtonClickActions.ForEach(btn => btn.Callback.RemoveAllListeners());
        _controllerButtonsMapper.ButtonClickActions.Clear();

        OVRPlugin.OVRP_1_83_0.mockObj = new OVRPlugin.OVRP_1_83_0_TEST();
        yield return null;
    }

    public override List<IEnumerator> TestCaseList => new()
    {
        TestShouldTriggerActionOnKeyPress()
    };

    private IEnumerator TestShouldTriggerActionOnKeyPress()
    {
        // On button up setup
        _testFixture.SetOVRInputState(OVRInput.Button.One);
        yield return null;

        _testFixture.SetOVRInputState(OVRInput.Button.None);
        yield return null;

        if (OVRInput.GetUp(OVRInput.Button.One))
        {
            // Expect exactly one key press.
            Assert.True(_onKeyUpCount == 1);
        }

        // On button down setup
        _testFixture.SetOVRInputState(OVRInput.Button.Two);
        yield return null;

        if (OVRInput.GetDown(OVRInput.Button.Two))
        {
            // Expect exactly one key press.
            Assert.True(_onKeyDownCount == 1);
        }

        yield return null;
    }

    private void SetupActions()
    {
        _onKeyUpCount = 0;
        _onKeyDownCount = 0;
        _controllerButtonsMapper = Utils.GetBlocksWithType<ControllerButtonsMapper>().First();

        var buttonAction1 = new ControllerButtonsMapper.ButtonClickAction
        {
            Title = "Action for controller button one",
            Button = OVRInput.Button.One,
            ButtonMode = ControllerButtonsMapper.ButtonClickAction.ButtonClickMode.OnButtonUp,
            Callback = new UnityEvent()
        };

        buttonAction1.Callback.AddListener(() => _onKeyUpCount++);
        _controllerButtonsMapper.ButtonClickActions.Add(buttonAction1);

        var buttonAction2 = new ControllerButtonsMapper.ButtonClickAction
        {
            Title = "Action for controller button two",
            Button = OVRInput.Button.Two,
            ButtonMode = ControllerButtonsMapper.ButtonClickAction.ButtonClickMode.OnButtonDown,
            Callback = new UnityEvent()
        };

        buttonAction2.Callback.AddListener(() => _onKeyDownCount++);
        _controllerButtonsMapper.ButtonClickActions.Add(buttonAction2);
    }
}

#endif //OVRPLUGIN_TESTING
