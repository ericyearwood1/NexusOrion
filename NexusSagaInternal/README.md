# Meta Fair Siro Phase H1

## Set up

![alt text](readme-assets/folder-structure.png "Folder Structure")

1. Create project folder  
2. Clone the `meta-fair-siro-phase-h1` project from bitbucket repository so the root folder sits inside your project folder  
3. Add the Saga Plugin folder (`SagaPackage`) to your project folder (the stub can be found zipped in the root of the git repository folder). This should, in turn, contain the `com.ix.saga` plugin.
4. Download the Unity Hub from [here](https://unity.com/download)). 
5. Once complete, open up the Hub and click on Install Editor (found at the top right). 
6. Select to install Unity version 2022.3.41f1 and make sure that you select to include Android build support when prompted.
7. Once the Unity Editor is installed, go back to Unity Hub and select Add.
8. Navigate to the `meta-fair-siro-phase-h1` folder and select Open. This will add the project to the list in your hub.
9. Open the Project Settings menu from Edit > Project Settings. Select `Player` in the left list and scroll to the bottom, to the `Publishing Settings` section. In the password field, type the password provided by the development team. The password needs to be entered twice, under `Project Keystore` and `Project Key`.
9. Next to your project name, click on the `2022.3.41f1` under Editor Version, make sure `Android` is ticked in the subsequent window, then click on Open with 2022.3.41f1
10. Once the project is open, navigate to File > Build Settings and click Build. This will ask you to select a build folder — if you do not already have a build one, click on `New Folder` to create one. Give your build a name, such as `siro-release` and click Save. This will build an apk to your build folder. Once complete, you can install the apk using `adb install`. 
11. If you would prefer, you can also build directly to device. To do so:
	- Connect your quest to your computer
	- Open Build Settings and select your Quest from the `Run Device` drop down (if you can't find your device, make sure it is set in Developer mode from the Meta app).
	- If patching the build, tick the Development Build checkbox (NB. we do not recommend you use this for final builds) and click Patch and Run
	- Or, if running a full build, click Build and Run

NB. You may need to add a key for the build to work. Meta have their own internal key for this purpose — please speak to Kavit Shah for information on where to get this. Once you have it, open Player Settings > Publish Settings and navigate to the key from the Keystore Manager (click on the dropdown and click on Select Existing).

## Permissions required

The app requires microphone and spatial data permissions. These should be requested when running the app, but if the user has refused permissions previously, you may have to do this manually. 

In your headset menu: 
Go to Settings > Privacy and safety > Device Permissions. Make sure hand tracking and spatial data is enabled.

Go to Settings > Privacy and safety > Permissions > Microphone. Find the SIRo app and make sure that Microphone is enabled.

Go to Settings > Privacy and safety > Permissions > Spatial Data. Find the SIRo app and make sure that Spatial Data is enabled.

## Device Set up

For each headset, please make sure they have spaces set up for each room being used. This can be done by navigating to:
Settings > Physical Space > Space Set up. Click set up to scan a new room. 

Before loading the application, please make sure to walk the space with each headset. This helps with loading/sharing spatial anchors. 

## Unity Editor Setup

Open the Main scene. In the Hierarchy window, under the `---Managers` GameObject, you will find the Root GameObject that initialises the demo. Click on this object and open the Inspector window. 
Here you will see several options for use:

- Saga Plugin Configuration : Normal = only enable for guest user; Always Enabled = enabled for all users (useful for testing if there is only one user in the experience, as they will have to be a Host); Always Disabled = disable the plugin for everyone. Default is Normal.
- IsTrackRobot : This will tell the app whether to track the UI above the robot, or not. The latter can help with testing the robot UI and action/feedback text. Default is True.  
- IsUseStubSiroService : If ticked, this will use a stub service will pre-configured instruction data when an instruction is sent. This is useful for testing the messages in expected sequence without the actual ROS services. Default is False.  
- UseServiceDiscovery: Auto-discovery for the siro service IP. Listens for a UDP broadcast from the SiroService containing the siro service IP. Default is currently False until we can properly test. Aim is for default to be True.
- Use Speech Recognition : Set to True to use the actual speech recognition flow, or False to send a dummy text instruction. The latter is useful for development speed. Default is True.
- Is Show Debug Objects : if checked, a green box will show at the robot pose position, and a red box at the EE pose position. This is useful for debugging robot drift.

If testing in Editor, you can use Link to test interaction from the headset, otherwise you can use tap Space to take you through most interactions. For secondary options, tapping the key for that corresponds to the first letter of the button label should work (i.e. to trigger a button with the label 'Edit' tap keyboard key 'E')

## Features

### User Details

Once you have connected to the server, you will be asked to fill in user details. On first run, you will be shown the Edit User Details panel, which will ask you to select the user type (see below), user color for the name display above the user's headset, and enter the username. Once filled out correctly, these details will be saved locally for future sessions.
If all the details are correct, the next time the application is loaded, the Confirm User details panel will be shown. If you wish to change user details at this point, you can click on the Edit button at the bottom. 

#### Troubleshooting

The user is required to be added to the app Release Channel or this stage will not work

#### User Types

There are three supported user types:
- Host : This user is responsible for setting up the experience. They will be asked to sync the robot space and place the world graph anchors. There should only be one Host user.
- Guest : This user should be the person you are demoing to. If the saga plugin is enabled, it will be the user that is tracked for the Human Activity Recognition service. There should only be one guest
- Observer: The purpose of this user is to stream video from the experience so it can be visible to others in the room. 


### UI

#### Full Focus UI

These are the panels that always sit in front of the user. They display when it is important that the user completes a task.

#### Hand UI (GameObject name = LeftHandMenuNew)

Once the user has progressed to the main state of the app, the controls are shown on a palm menu. 
To adjust the position of the menu, in the Unity Editor, please change the Local Space Offset and/or Rotation offset values in the Inspector window.

##### Manual Controls Panel

This was originally intended as a Host-only menu, but is now available for all users. Features:

- Stop Instruction : Send a message to the planner to stop the current instruction
- World Graph Visualization : ON/OFF: Toggles the World Graph visualisation app feature. Please see the App Features section below
- Manual Instructions: ON/OFF: Toggles the Manual Instructions app features. Please see the App Features section below


### Service Connection and Communication

The ConnectToSiroServiceState handles service discovery and initiating connection, and the SiroService is responsible for the connection itself. The server broadcasts an auto-discovery message containing its IP address. Once this message has been read, the service will automatically connect to the service.


#### Troubleshooting
We have seen issues with the auto connection not sending the correct ip address. In these cases, the solution was to make sure the users were on the same network. In the worst instance, the best solution is to hardcode the ip in the broadcast message on the server. Alternatively, auto-discovery can be disabled on the Root game, please see Unity Setup for more details.

The service uses the following external packages for communication and serialization/deserialization:

- WebSocketSharp.CustomHeaders.CustomHttpServer.1.0.2.3
- EngineIOSharp.1.0.5
- EmitterSharp.1.1.1.1
- Newtonsoft.Json-for-Unity.Converters


### Messages

To send a message to the service, implement the IClientMessage interface and use ISiroService.SendMessage(endpoint, clientMessage)

To send a request to the service (i.e a request for the latest World Graph data), use ISiroService.SendRequest(endpoint)

ServerMessages consist of the following:
- EventName: the service that send the message. 
- EventType: the type of message being sent
- Data: a DTO object. The type of this object is directly related to the EventType

#### Valid EventNames:
- "skills": Skills service. Sends highlight messages to help indicate what the robot is doing. Please see the Highlights section below.
- "multiplayer": Sends user join/user leave and a message that helps to track the user name display
- "planner": Sends information about the current instruction (replanning/ instruction complete etc). Please see the Robot Display section below
- "world_graph": Sends world graph data and edits. Please see the World Graph Visualisation below
- "human_activity": Please see the Saga Plugin and Human activity section below
- "siro": Information from the Siro Service itself. Please see the Manual Instruction section below.
- "speech": Communication from the speech service, regarding STT results. 

Please see the separate readme-assets/server-messages folder to see the supported messages

#### Message Handlers

Each EventType has its own IMessageHandler. This handles the deserialization of the message according to its EventType, and also processes the resulting DTO. 


### Notification System

This system allows notifcations to be shown to the user, so they can see important information wherever they are in the room. Currently this system is used to show when a user has joined or left, when an instruction has been completed by the planner and any Human activity updates.


### Saga Plugin and Human Activity Recognition (HAR)

This is an internal Meta plugin that sends camera and depth images, as well as the camera matrix to the human activity service. These are used by the service to determine what the user is doing. When an activity is recognised, the service sends a "human_activity" message containing an 'activity' field, which is displayed in a notification for the user to see. Human Activity Messages are processed by the HumanActivityMessageHandler. 
You can configure the activation of the Saga plugin by changing the Saga Plugin Configuration option on the Root gameobject in the Unity Editor. Please see Unity Setup above for more information.

To use the saga plugin correctly, Meta's internal XR packages must be used. Please speak to Kavit Shah on how to do this. Once added, you must make sure that version 1 Hand Tracking is set (this is already set in the OVR Manager config, but the internal packages show another field that should be set): Open the Main Scene > In the Hierarchy, expand ---XR and select OVRCameraRig > In the Inspector window, look for a hand tracking or skeletal tracking drop down and make sure that V1 is selected. 

### Robot Display

This is a panel that tracks over the robot. It will display messages regarding the current action, as well as the current state of the planner (replanning etc). 
This can be found in the Main Scene at 
---RobotSync > RobotHome > RobotDisplay > RobotUI


### App Features

These are features that can be toggled on and off for all users from the Hand UI. The state of the feature (ON/OFF) is stored on the server. The state is wiped once all users have left the room or the service is restarted.

The current features available are listed in AppFeatures.cs
- MANUAL_STT_INSTRUCTION_PANEL = "Manual STT Panel";
- WORLD_VISUALISATION = "World Visualization";

The AppFeatureHandler stores each available feature alongside a list of Handlers for processing different behaviour when the feature is activated/deactivated. 

New Feature Handlers can be added to the system using FeatureActivationHandler.AddFeature(FEATURE_NAME, FEATURE_HANDLER);

Examples of current handlers are:
- HARActivationSystem: if the "har" field in the feature config == 1 (the default) then this will activate the HAR notifications on this feature toggle
- WorldGraphVisualisationSystem: shows/hides the world graph visuals on this toggle
- ManualInstructionPanelVisualisationSystem: shows/hides the manual instruction panels on this toggle. These will show beside the robot home and in the Hand UI.

Please see readme-assets/server-messages/multiplayer.md for more details on the message structure

#### Manual Instructions

This is extra UI that can be displayed if the user is having trouble using the STT service. 
The instructions are received from the server on connection (please see server readme for more details).
This can be toggled on from the hand UI manual control panel. Pressing this toggle will change the state for all users. 


#### World Graph Visualisation

You can configure which objects show as flat, or 3d by adding/removing the relevant object_name to the FlatDisplayList field in Assets/Config/World Graph Object Config.asset. Currently, items with the following object_names are configured to show as a flat mesh : 
table
console
counter
island
cabinet

Use Object Name or Object Id for furniture highlight labels : The IsUseObjectIdForFurnitureLabel toggle in the Assets/Config/World Graph Object Config.asset to configure this.

The following are the object_names that can be displayed in the world graph panel/highlights. Each object requires an equivalent icon to be added to Unity, so please let the team know if there is anything missing.

office_chair
coffee_table
bed
party_hat
donut
cup
can
bottle
kitchen_counter
kitchen_island
dining_table
sofa
wooden_console
refrigerator
wooden_dresser
cabinet
drawer
night_stand
trash_can
pillow
hamper
shelf
chair
ball
toy

##### Add an icon to the World Graph

To add a new icon, first determine if it's an 'object' or 'furniture' item, as these have slightly different looks.

Furniture Icons  
Furniture icons can be found here : Assets/Textures/UI/Figma/furniture
The height should be 220px, but the width is variable. The icon should have an alpha of 40% and a fully visible outline. 

Object Icons  
Object icons can be found here : Assets/Textures/UI/Figma/object
They should be the same width and height and have be fully opaque and white

Icons are mapped in the config asset found here : Assets/Config/World Graph Object Config.asset

##### World Graph Object Config

Flat Display List  
If an object name contains an item in this list, the 3d bounding box in the world graph visualisation will display as flat

Mappings  
This is where you should add an icon mapping
Id = the object_name that you want to map the icon to
Icon = the sprite icon
Label = deprecated field


### Highlights

Each type of highlight has its own HighlightSystem<ViewObject, DTO> 
i.e. PlaceLocationHighlightSystem : HighlightSystem<PlaceLocationMarker, PlaceLocationData>

The DTO must inherit from HighlightData, which contains a Position field to set location of the highlight 

Listeners are set up in the RobotCommandState and OnHighlightReceived/HideAllMarkers functions are called on the correct highlight system at the correct time
The simplest highlight system is probably the PlaceLocationHighlightSystem — if a new highlight system is required, simply follow the structure of the prefab and the system. There are more complex examples, such as the SemanticPlaceHighlightSystem which tracks the highlight with the EE pose.


### State Machine

We use StateKit for the application StateMachine. You can find the StateMachine initialisation in Root.InitialiseStateMachine(). 


## Feature Configuration

"har": The human activity configuration — 0 = always enabled, 1 = only show when world graph visualisation shows, 2 = always disabled
"is_notifications_active": global switch for notifications
"is_show_place_object_highlight": global switch for whether to show the place object highlights
"is_show_semantic_place": global switch for whether to show the semantic place highlight
"is_show_world_object_highlights": global switch for whether to show the object label highlights in the world graph visualisation
"is_show_target_object_highlight": global switch for whether to show target object highlights
"saga_ip": from build 0.1.50. The IP address for the saga plugin to send data to



## Colors

You can change the colors used for the world graph visualisations and the normal highlights on the Root gameobject. Open the Inspector with the Root object selected, and at the bottom you should see 
World Graph Highlight Colour : the tint of the world graph highlights. Defaults to white
Normal highlight Color : the tint of all the normal highlights. Defaults to light blue
Semantic Place Target Arrow Color : the color of the target arrow on the semantic place highlight. Defaults to green. Please pay attention to maintaining the alpha if changing this field.


## Anchors

NB. Never select to override anchors when other users are in the experience, this is not supported.

First run through : 

1. Host should set up the experience fully
2. Once happy with the positions of all the anchors, set up the other headsets
3. The anchors will be persisted on the server, across sessions. If you move locations or want to set up the experience again, the cleanest way is to delete the server save (on the server : siro_server/config/multiplayer-state.pkl)
4. Set up the other user headsets. For the first run through, the HOST MUST be in the experience to share anchors with other users. Once the anchors have been successfully shared once, the HOST no longer has to be in the experience 
5. When the Host has shared an anchor with anothe user, that user's id will be added to the shared_with field in the anchor data and an anchor_update message will be broadcast to all connected users 
6. When the other user receives anchor data and sees that it is shared with them, if the local anchor state is not currently set to Ready, the system will attempt to load that anchor. 
7. Once an anchor is loaded (or saved) the anchor is available to load locally from then on. The anchor id is saved to the app Player Prefs. When the user loads the application again, it will check to see if any requested anchors are stored in the player prefs, and if so, a local load is requested. If this fails, the system will fallback to a load from the cloud

Known issues : light in the room can impact anchor loading. 

### If issues, try:
1. Delete the save file at siro_server/config/multiplayer_state.pkl
2. If for whatever reason, anchors fail to load, reopen the app and try again. 
3. Make sure the host is in the app until the anchors have successfully been shared with the other users. You can check the server logs for this (anchors are stored in room_state)

### Anchor Types

- CoLocation: 
- RobotHome: 
- WorldGraph: 

#### Colocation 

Placed automatically on a fresh run, or if the Host has decided to override anchors. This is placed at the Hosts origin position and used to sync spaces with other users. Once loaded, the colocation home object is set to the same pose as the loaded anchor

#### Robot Home

Once the host has finished the three point sync with the robot, the robot home anchor is saved and shared with other users. The robot_home gameobject is set to match the pose of this anchor


#### World Graph Panels

This feature uses the OVR Spaces functionality, so you will need to set up your spaces in Settings> Physical Space > Space Set up, then click Set up.

1. The host will need to place a panel for each room in the world graph. 
2. Use your right hand to position the panel, and confirm by pinching with your left hand. You can do this again to edit the position. Once happy, click the forward arrow. 

An anchor is saved for each panel and shared with the other users. This is to mitigate any accuracy issues with sharing across multiple spaces


##### Troubleshooting
The Host MUST be online to share anchors with other users. If they are not online, the user will be stuck at this panel

If the host is online and the user is stuck on Co-Locating, it's typically because the headset is having issues loading the anchors (it doesn't have enough data to place them) or the Host headset is having trouble sharing them. Close the app and walk the headset around the room, then reopen the application.

Anchors not positions correctly, or at the incorrect orientation. We have seen this when the headset saving the anchor, or the headset loading the anchor loses tracking of the space it is in. If the former, close the app, walk the headset around the room. Recreate the room spaces from the headset settings menu. Then either wipe the multiplayer_state.pkl file and start from fresh, or open the application and choose to override anchors.


## Move the application to another Release Channel

1. Go to https://developers.meta.com/
2. Create a new application
3. If it doesn't open automatically, open the application by clicking on the icon in the dashboard
4. Request the necessary permissions. Go to Data Use Checkup and enable User Id and User Profile. This is needed to access the user's profile. You can select that it's required for multiplayer use, but it doesn't matter very much here as this is not a publically available application.
5. Find the app details in Development > API and copy the App Id and App Secret
6. Open the Unity project in the Unity Editor
7. Navigate to Meta > Platform > Edit Settings (this can also be found in Resources > OculusPlatformSettings)
8. Open Meta > Tools > OVR Platform Tools
9. Upload a build to the release channel : https://developers.meta.com/horizon/documentation/unity/unity-platform-tool/
10. You can see the release build in the channel at https://developers.meta.com/, then navigate to the application and go to Distribution > Release channels
11. Click on Users
12. Click Email invite users and add the users associated with the headsets you want to to test with
13. You can also click on Channel Settings and select to share via URL (this is sometimes easier as it can be confusing for users to find the correct email associated with their account)

