var dgram = require('dgram');


function writeHeartbeatData(agentID, count) {
    let message = {
        message_type: "vehicle_heartbeat",
        agentID: agentID,
        autopilot: "AUTOPILOT_TYPE_ARDUPILOTMEGA",
        vehicle_type: "FIXED_WING",
        companion: false,
        protocol: "COMMS_MAVLINK",
        mission_state: 0,
        mavlink_id: 0,
        behavior_state: "",
        vehicle_state: "Flight Guided Idle"
    }

    writeUDPData(JSON.stringify(message));
}

function writePositionData(agentID, count) {
    let message = {
        message_type: "vehicle_position",
        agentID: agentID,
        alt: 10,
        lat: -35.3632621 + count/10000,
        lng: 149.1652374 + (parseFloat(agentID) / 300)
    }

    writeUDPData(JSON.stringify(message));
}

function writeAttitudeData(agentID, count) {
    let message = {
        message_type: "vehicle_attitude",
        agentID: agentID,
        pitch: 0.0 + (count/ 10),
        roll: 0.0 + (count/ 10),
        yaw: 0.0 + (count)
    }

    writeUDPData(JSON.stringify(message));
}

function writeVehicleAirspeed(agentID, count) {
    let message = {
        message_type: "vehicle_airspeed",
        agentID: agentID,
        airspeed: 0.0 + (count/ 1000)
    }

    writeUDPData(JSON.stringify(message));
}

function writeVehicleGPS(agentID, count) {
    let message = {
        message_type: "vehicle_gps",
        agentID: agentID,
        gps_fix: "RTK Fixed",
        vdop: 10,
        hdop: 10,
        visible_sats: 10 + (count/ 1000),
    }

    writeUDPData(JSON.stringify(message));
}

function writeVehicleMode(agentID, count) {
    let message = {
        message_type: "vehicle_mode",
        agentID: agentID,
        mode: "GUIDED"
    }

    writeUDPData(JSON.stringify(message));
}

function writeVehicleFuel(agentID, count) {
    let message = {
        message_type: "vehicle_fuel",
        agentID: agentID,
        battery_current: count/1000,
        battery_remaining: count/1000,
        battery_voltage: count/1000
    }

    writeUDPData(JSON.stringify(message));
}

function writeVehiclePath(agentID, count) {
    let message = {
        message_type: "vehicle_path",
        agentID: agentID,
        vertices: [
            {lat: -35.3632621, lng: 149.1652374},
            {lat: -35.3632621 + parseFloat(agentID)/10, lng: 149.1652374 + parseFloat(agentID)/100},
            {lat: -35.3632621 + parseFloat(agentID)/100, lng: 149.1652374 + parseFloat(agentID)/1000},
            {lat: -35.3632621 + parseFloat(agentID)/1000, lng: 149.1652374 + parseFloat(agentID)/10000},
        ]
    }

    writeUDPData(JSON.stringify(message));
}

function writeVehicleTarget(agentID, count) {
    let message = {
        message_type: "vehicle_target",
        agentID: agentID,
        location: {lat: -35.3632621, lng: 149.1652374},
        is_global: false,
        distance_to_target: count/10
    }

    writeUDPData(JSON.stringify(message));
}


function writeUDPData(data) {
    var PORT = 8080;
    var HOST = '127.0.0.1';

    var message = new Buffer(data);
    var client = dgram.createSocket('udp4');
    client.send(message, 0, message.length, PORT, HOST, function(err, bytes) {
      if (err) throw err;
    //   console.log('UDP message sent to ' + HOST +':'+ PORT);
      client.close();
    });
}


module.exports = { writeHeartbeatData, writeAttitudeData, writePositionData, writeVehicleAirspeed, writeVehicleGPS, writeVehicleMode, writeVehicleFuel, writeVehiclePath, writeVehicleTarget };




