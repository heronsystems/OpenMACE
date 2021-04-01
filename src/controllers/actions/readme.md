A controller is to contain a set of actions that define how the controller responds to messages receiving over the communication protocol.
Developing actions is the bulk of the development requements when building a new controller: deciding what actions are needed, how they interact, and how they share data.
Any number of actions can be assigned to a controller.
Some controllers, such as commands, only need three actions on both sides: Initiate (sender), Receive&ACK (receiver), and Finalize (sender).
Other controllers, like boundary management, need many actions to control all the transmissions needed to deliver a boundary across the communication protocol.

Bellow is a table of available actions, a short description, and an example on how it may be used.

| Action            | Description | Example |
   |------------------|---------|-------------|
   | [ActionBroadcast](https://github.com/heronsystems/MACE/blob/documentation-and-simplification/src/controllers/actions/action_broadcast.h) | Unreliable transmit of data to all entities |Sending position updates |
   | ActionBroadcastReliable   | Executes a single broadcast of data to all entities, expecting a reply of some kind. If that reply isn't heard will resend tranmission to the entities that didn't respond as expected. Can reduce overall communication when multiple targets need to get the same message | Notifying all MACE instances on the existance of a new boundary |
   | ActionBroadcastReliable_MultiPacket | Same as above, but can generate multiple messages per data object. Usefull if multiple packets are needed to represent some datatype. | Splits a BoundaryCharacterstic into multiple messages, one for each vehicle a boundary applies to |
   | [ActionSend](https://github.com/heronsystems/MACE/blob/documentation-and-simplification/src/controllers/actions/action_send.h) | Initiate by doing reliable send of data to a single target |Commanding vehicle |
   | [ActionIntermediateUnsolicited](https://github.com/heronsystems/MACE/blob/documentation-and-simplification/src/controllers/actions/action_intermediate_unsolicited.h) | Initiates a "handshake" communication protocol. Receives an unexpected message, generates reliable response | Start boundary upload procedure: Upon receiving `boundary_request_list` respond with `boundary_count` |
   | [ActionIntermediate](https://github.com/heronsystems/MACE/blob/documentation-and-simplification/src/controllers/actions/action_intermediate.h) | Continues a "handshake" communication protocol. Receives an expected message, dequeues previous transmission, generates reliable response.  | Continues mission upload procedure: Upon receiving `item_request` respond with `mission_item` | 
   | [ActionUnsolicitedReceive](https://github.com/heronsystems/MACE/blob/documentation-and-simplification/src/controllers/actions/action_unsolicited_receive.h)[^1] | Receive an unknown message, deliver data to host, and generate NO response to the sender | Process a broadcasted command |
   | [ActionUnsolicitedReceiveResponse](https://github.com/heronsystems/MACE/blob/documentation-and-simplification/src/controllers/actions/action_unsolicited_receive_respond.h) | Receive an unexpected message, deliver data to host, and respond unreliably | Receiving a command: Alert host of command, and send back ACK confirming the command has been done |
   | [ActionFinalReceiveResponse](https://github.com/heronsystems/MACE/blob/documentation-and-simplification/src/controllers/actions/action_final_receive_respond.h) | Receive an expected message, dequeue previous transmission, deliver data to host, and respond unreliably  | Upon receiving last mission item, deliver full mission to host and respond with ACK to uploader  |
   | [ActionFinish](https://github.com/heronsystems/MACE/blob/documentation-and-simplification/src/controllers/actions/action_finish.h) | Finalizes upload procedure. Receive an expected message, dequeue previous transmission, deliver ack to host and finish. | Receive ACK upon commanding vehicle |
   |

[^1]
Such action is likely deprecated. Almost all commands should generate an ACK, thus ActionUnsolicitedReceiveResponse should be used instead
