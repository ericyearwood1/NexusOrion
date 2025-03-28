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

namespace Meta.XR.BuildingBlocks.Editor
{
    internal static class BlockDataIds
    {
        public const string CameraRig = "e47682b9-c270-40b1-b16d-90b627a5ce1b";
        public const string ControllerTracking = "5817f7c0-f2a5-45f9-a5ca-64264e0166e8";
        public const string EyeGaze = "f92445a7-db19-452c-8d3d-96820f6c8972";
        public const string HandTracking = "8b26b298-7bf4-490e-b245-a039c0184303";
        public const string Passthrough = "f0540b20-dfd6-420e-b20d-c270f88dc77e";
        public const string RoomModel = "be2b0240-1191-4e84-90bb-40fb6d75848b";
        public const string PassthroughOverlay = "40e08c51-14aa-4822-927d-5fe79943b5b4";
        public const string SurfaceProjectedPassthrough = "19c06269-24b5-4657-a433-21a6f80dbabf";
        public const string VirtualKeyboard = "bac08eda-400f-4f61-8842-77e7699e068d";
        public const string PassthroughWindow = "4f06f550-8209-46c8-a4c9-879a368560f6";
        public const string SampleSpatialAnchorController = "1f4566e8-f4ba-46d2-9e47-3748c9869766";
        public const string SpatialAnchorCore = "a383f5ea-3856-4c23-a11c-7fdbb9408035";
        public const string SharedSpatialAnchorCore = "975428dc-db04-4e43-be4a-232f192cd3eb";
        public const string ControllerButtonsMapper = "80aa2eb2-af2f-40a9-ad9e-18044ce65bee";

        // Utilities
        public const string Cube = "7358e191-33b7-43ca-b066-b2f00531ac66";
        public const string AlertViewHUD = "dfd47015-f5ae-4435-8b15-8acde41b3871";

        // Interaction
        public const string GrabbableItem = "5c5184f2-c2f5-4063-b14b-3b1264fb3c1a";
        public const string PointableItem = "76a013d1-7c16-4a60-9c1b-79b3691c0438";
        public const string PokeableItem = "5838e892-31a4-4f65-a415-dc108aefa14c";
        public const string RealHands = "f547fe18-d477-46ec-bdf5-7208df19cb98";
        public const string SyntheticHands = "6b67162c-2460-4766-a931-980388647573";
        public const string ThrowableItem = "803f3803-bb5e-464a-ac45-dad44c840fb6";

#if OVR_INTERNAL_CODE
        public const string PassthroughSettings = "a18af262-5790-4a70-9927-15826d9bcbed";
        public const string RoomMesh = "6b8a377d-05de-449b-a9c4-8ed22606f71c";
        public const string PlaceObjectOnRoomMesh = "d08c557e-a09a-4c39-9efe-57f1c8f76fd2";
        public const string DynamicOcclusion = "c1dbe32c-fed2-4835-8e6a-3fbb4f4f4315";
        public const string RemoteSDK = "22d3f7a2-4ab0-4ff7-8d34-4d8bf15bfcc0";
#endif // OVR_INTERNAL_CODE
    }
}
