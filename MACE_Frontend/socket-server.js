// const net = require("net");
// const websocket = require("ws");

// /** These are added to the global namespace in main.js (electron) */
// const { mainWindow, secondaryWindow } = global;
// const sendToGUI = (message) => {
//   const channel = "server";
//   mainWindow.webContents.send(channel, message);
//   // secondaryWindow.webContents.send(channel, message);
// };

// // let latitudeA = 38.2845470188316;
// // let latitudeB = 38.4;
// // let latitudeC = 38.6;
// // let latitudeD = 38.8;
// // let latitudeE = 39.0;

// // let longitudeA = -76.7;
// // let longitudeB = -76.5;
// // let longitudeC = -76.3;
// // let longitudeD = -76.1;
// // let longitudeE = -75.9;
// // global.interval = setInterval(() => {
// //   let messageA = {
// //     message_type: "vehicle_heartbeat",
// //     agentID: "agent_123",
// //     fidelity: "real",
// //     autonomy_state: "",
// //     communication_state: "transmitting",
// //     vehicle_state: "",
// //     type: "slow",
// //     behavior_state: "",
// //     remediation_state: false,
// //     orientation: {
// //       pitch: 0,
// //       roll: 0,
// //       yaw: 0
// //     },
// //     location: { lat: latitudeA, lng: longitudeA }
// //   };
// //   let messageB = {
// //     message_type: "vehicle_heartbeat",
// //     agentID: "agent_124",
// //     fidelity: "real",
// //     autonomy_state: "",
// //     communication_state: "transmitting",
// //     vehicle_state: "",
// //     type: "slow",
// //     behavior_state: "",
// //     remediation_state: false,
// //     orientation: {
// //       pitch: 0,
// //       roll: 0,
// //       yaw: 0
// //     },
// //     location: { lat: latitudeB, lng: longitudeB }
// //   };
// //   let messageC = {
// //     message_type: "vehicle_heartbeat",
// //     agentID: "agent_125",
// //     fidelity: "real",
// //     autonomy_state: "",
// //     communication_state: "transmitting",
// //     vehicle_state: "",
// //     type: "slow",
// //     behavior_state: "",
// //     remediation_state: false,
// //     orientation: {
// //       pitch: 0,
// //       roll: 0,
// //       yaw: 0
// //     },
// //     location: { lat: latitudeC, lng: longitudeC }
// //   };
// //   let messageD = {
// //     message_type: "vehicle_heartbeat",
// //     agentID: "agent_126",
// //     fidelity: "real",
// //     autonomy_state: "",
// //     communication_state: "transmitting",
// //     vehicle_state: "",
// //     type: "slow",
// //     behavior_state: "",
// //     remediation_state: false,
// //     orientation: {
// //       pitch: 0,
// //       roll: 0,
// //       yaw: 0
// //     },
// //     location: { lat: latitudeD, lng: longitudeD }
// //   };
// //   let messageE = {
// //     message_type: "vehicle_heartbeat",
// //     agentID: "agent_127",
// //     fidelity: "real",
// //     autonomy_state: "",
// //     communication_state: "transmitting",
// //     vehicle_state: "",
// //     type: "slow",
// //     behavior_state: "",
// //     remediation_state: false,
// //     orientation: {
// //       pitch: 0,
// //       roll: 0,
// //       yaw: 0
// //     },
// //     location: { lat: latitudeE, lng: longitudeE }
// //   };
// //   sendToGUI(JSON.stringify(messageA));
// //   sendToGUI(JSON.stringify(messageB));
// //   sendToGUI(JSON.stringify(messageC));
// //   sendToGUI(JSON.stringify(messageD));
// //   sendToGUI(JSON.stringify(messageE));

// //   latitudeA += 0.001;
// //   latitudeB += 0.001;
// //   latitudeC += 0.001;
// //   latitudeD += 0.001;
// //   latitudeE += 0.001;
// //   longitudeA += 0.001;
// //   longitudeB += 0.001;
// //   longitudeC += 0.001;
// //   longitudeD += 0.001;
// //   longitudeE += 0.001;
// // }, 5);

// // Avoid dead sockets by responding to the 'end' event
// const sockets = [];
// var message = ""; // variable that collects chunks
// var message_separater = "\r";
// // Create a TCP socket listener
// const server = net.Server(function (socket) {
//   // Add the new client socket connection to the array of
//   // sockets
//   sockets.push(socket);
//   console.log("Added new socket", sockets.length);
//   // 'data' is an event that means that a message was just sent by the
//   // client application
//   socket.on("data", function (data) {
//     console.log("Open socket connections: ", sockets.length);

//     message += data;

//     let message_separater_index = message.indexOf(message_separater);
//     let foundEntireMessage = message_separater_index != -1;

//     if (foundEntireMessage) {
//       var msg = message.slice(0, message_separater_index);

//       // Send to GUI:
//       sendToGUI(msg);

//       // Respond to client for proper closing:
//       msg += message_separater;
//       sockets.forEach((s) => {
//         s.write("GUI received message, closing socket.\r");
//       });

//       message = message.slice(message_separater_index + 1);
//     }

//     // sockets.forEach((s) => {
//     //   console.log(data.toString());
//     //   s.write(data);
//     //   sendToGUI(data.toString());
//     // });
//   });

//   // Use splice to get rid of the socket that is ending.
//   // The 'end' event means tcp client has disconnected.
//   socket.on("end", function () {
//     // console.log("socket done");
//     // const i = sockets.indexOf(socket);
//     // sockets.splice(i, 1);
//   });
//   socket.on("close", function () {
//     // console.log("Trying to close");
//     const i = sockets.indexOf(socket);
//     sockets.splice(i, 1);
//   });
//   socket.on("error", function (err) {
//     console.log("Error with socket: ", err);
//   });
// });
// server.listen(8080);
// console.log("System waiting at http://localhost:8080");
