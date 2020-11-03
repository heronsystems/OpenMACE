var PORT = 8080;
var HOST = '127.0.0.1';

// var udp = require("./udp_helper");
const udp = require("./udp_helper");

let numAgents = 10;
let agentIDs = [];
for(let i = 1; i < numAgents+1; i++) {
    agentIDs.push(i);
}

function shuffle(array) {
    for(let i = array.length - 1; i > 0; i--){
        const j = Math.floor(Math.random() * i)
        const temp = array[i]
        array[i] = array[j]
        array[j] = temp
    }

    return array;
}


function ping() {

    // // Loop over all agents in order:
    // for(let i = 1; i < numAgents+1; i++) {
    //     udp.writeHeartbeatData(i.toString(), count);
    //     udp.writePositionData(i.toString(), count);
    //     udp.writeAttitudeData(i.toString(), count);
    //     udp.writeVehicleAirspeed(i.toString(), count);
    //     udp.writeVehicleGPS(i.toString(), count);
    //     udp.writeVehicleMode(i.toString(), count);
    //     udp.writeVehicleFuel(i.toString(), count);
    //     udp.writeVehicleTarget(i.toString(), count);
    //     count++;
    // }

    // Loop over all agents in RANDOM order:
    let agents = shuffle(agentIDs);
    agents.forEach(agent => {
        udp.writeHeartbeatData(agent.toString(), count);
        udp.writePositionData(agent.toString(), count);
        udp.writeAttitudeData(agent.toString(), count);
        udp.writeVehicleAirspeed(agent.toString(), count);
        udp.writeVehicleGPS(agent.toString(), count);
        udp.writeVehicleMode(agent.toString(), count);
        udp.writeVehicleFuel(agent.toString(), count);
        udp.writeVehicleTarget(agent.toString(), count);
    });
}

let count = 0;
setInterval(() => {
    ping();
    count++;
    console.log("Message count: " + count)
}, 100);






