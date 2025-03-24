# World Graph Messages 

EventNme="world_graph"


## World Graph

EventType = "world_graph"

Initial world graph message. All current nodes and edges

### DTO

"graph": WorldGraph


## Edit message

EventType = "edit"

World graph edit message. When a node is added/removed or changes receptacle

### DTO - WorldGraphNodeEditDTO

"object": WorldGraphNodeEdit - the node that has been edited
