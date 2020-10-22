var PORT = 8080;
var HOST = '127.0.0.1';

// var udp = require("./udp_helper");
const udp = require("./udp_helper");

let numAgents = 10;

function ping() {
    for(let i = 1; i < numAgents+1; i++) {
        udp.writeHeartbeatData(i.toString(), count);
        udp.writePositionData(i.toString(), count);
        udp.writeAttitudeData(i.toString(), count);
        // udp.writeVehicleAirspeed(i.toString(), count);
        // udp.writeVehicleGPS(i.toString(), count);
        // udp.writeVehicleMode(i.toString(), count);
        // udp.writeVehicleFuel(i.toString(), count);
        // udp.writeVehicleTarget(i.toString(), count);
        count++;
    }
}

let count = 0;
setInterval(() => {
    ping();
    count++;
    console.log("Message count: " + count)
}, 100);






