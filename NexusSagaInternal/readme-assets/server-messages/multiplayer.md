# Multiplayer Messages 

EventNme="multiplayer"


## User Joined

EventType = "user_join"

### DTO

"user": User — all user details for the user that has joined

## User Left

EventType = "user_left"

### DTO

"user_id": ulong - the id of the user that has left

## Initial State

EventType = "initial_state"

### DTO

"sid": The server sid
"rooms": A list of available rooms. The server is only set up for one room currently
"config": FeatureConfiguration - app configuration driven from the server. See Feature Configuration in the main README for more info.

## Room State

EventType = "room_state"

### DTO

"room": Room - The current room state. Contains anchor data, active features, connected users

## Synced Entity  

EventType = "synced_entity"

Used for tracking multiplayer entities. Only one currently supported is the User Details panel

### DTO

"room": str - the room that the entity belongs to
"entity_id" — the unique id of the entity
"position" - the local position of the entity relative to the colocation home gameobject
"rotation" - the local rotation of the entity relative to the colocation home gameobject
"scale" - the local scale of the entity relative to the colocation home gameobject

## Spatial Anchor Message

EventType = "spatial_anchor_update"

Sent when an anchor has been saved or shared

### DTO 

"room": str - the room that the entity belongs to
"anchor_data":SpatialAnchorData - the details of the updated anchor

### Activate Feature

EventType = "activate_feature"

### DTO

"feature": The feature that has been activated

### Deativate Feature

EventType = "deactivate_feature"

### DTO

"feature": The feature that has been deactivated

