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

using Meta.XR.ImmersiveDebugger.Manager;
using Meta.XR.ImmersiveDebugger.UserInterface.Generic;
using System.Collections.Generic;

namespace Meta.XR.ImmersiveDebugger.UserInterface
{
    public class Values : Controller
    {
        protected List<Value> _values = new List<Value>();

        public Watch Watch { get; private set; }

        public ImageStyle BackgroundStyle
        {
            set
            {
                foreach (var valueItem in _values)
                {
                    valueItem.BackgroundStyle = value;
                }
            }
        }

        public TextStyle TextStyle
        {
            set
            {
                foreach (var valueItem in _values)
                {
                    valueItem.TextStyle = value;
                }
            }
        }

        public void Setup(Watch watch)
        {
            if (watch != Watch)
            {
                Watch = watch;

                for (var i = 0; i < Watch.NumberOfValues; i++)
                {
                    var value = Owner.Append<Value>("value");
                    value.LayoutStyle = Style.Load<LayoutStyle>("MemberValue");
                    value.TextStyle = Style.Load<TextStyle>("MemberValue");
                    value.BackgroundStyle = Style.Load<ImageStyle>("MemberValueBackground");
                    _values.Add(value);
                }

                Owner.Owner.Owner.RefreshLayout();
            }
        }

        public void Update()
        {
            if (Watch == null) return;

            var watchValues = Watch.Values;

            var index = Watch.NumberOfValues;
            foreach (var value in _values)
            {
                value.Content = watchValues[--index];
            }
        }
    }
}

#endif // OVR_INTERNAL_CODE
