import * as deepcopy from "deepcopy";
const net = electronRequire("net");
import * as GlobalTypes from "../../types/globalTypings";
import { VehicleDB } from "../Vehicle/VehicleDB";

export class MACECommsHelper {
    tcpServer: any;
    tcpSockets: any[];
    MACEconfig: GlobalTypes.ConfigSettingsType;
    vehicleDB: VehicleDB;

    constructor(MACEconfig: GlobalTypes.ConfigSettingsType) {
        this.MACEconfig = MACEconfig;
        this.tcpServer = null;
        this.tcpSockets = [];
        this.vehicleDB = new VehicleDB();
    }

    setupTCPServer = () => {
        // Close server if already exists:
        if (this.tcpServer !== null) {
            this.tcpServer.close();
        }

        // Close all existing sockets:
        let tcpSockets = deepcopy(this.tcpSockets);
        for (let i = 0; i < tcpSockets.length; i++) {
            tcpSockets[i].destroy();
        }
        this.tcpSockets = [];

        // Create a TCP socket listener
        let tcpServer = net.createServer(
            function(socket: any) {
                let remoteAddr = socket.remoteAddress.replace(/^.*:/, "");
                if (remoteAddr === this.MACEconfig.config.MACEComms.ipAddress) {
                    // Add the new client socket connection to the array of sockets
                    this.tcpSockets.push(socket);

                    // 'data' is an event that means that a message was just sent by the client application
                    socket.on(
                        "data",
                        function(msg_sent: any) {
                            // console.log("Data from socket: " + msg_sent);
                            let jsonData: GlobalTypes.TCPReturnType = JSON.parse(msg_sent);
                            // if(jsonData.dataType === "VehicleMission") {
                            //     console.log("Data from socket: " + msg_sent);
                            //     console.log(jsonData);
                            // }
                            this.vehicleDB.parseJSONData(jsonData);
                        }.bind(this)
                    );
                    // Use splice to get rid of the socket that is ending.
                    // The 'end' event means tcp client has disconnected.
                    socket.on(
                        "end",
                        function() {
                            // let sockets = deepcopy(this.tcpSockets);
                            let i = this.tcpSockets.indexOf(socket);
                            this.tcpSockets.splice(i, 1);
                        }.bind(this)
                    );
                }
            }.bind(this)
        );

        this.tcpServer = tcpServer;

        try {
            // this.tcpServer.listen(this.MACEconfig.config.MACEComms.listenPortNumber);
            this.tcpServer.listen(this.MACEconfig.config.MACEComms.listenPortNumber, this.MACEconfig.config.MACEComms.ipAddress);
        } catch (e) {
            console.log(e);
        }

        console.log("System listening at http://" + this.MACEconfig.config.MACEComms.ipAddress + ":" + this.MACEconfig.config.MACEComms.listenPortNumber);
    };

    makeTCPRequest = (vehicleID: number, tcpCommand: string, vehicleCommand: string) => {
        // Log message:
        // this.logger.info("{TCP Command: " + tcpCommand + "}  {Vehicle Command: " + vehicleCommand + "}");

        let socket = new net.Socket();
        this.setupTCPClient(socket);
        socket.connect(
            this.MACEconfig.config.MACEComms.sendPortNumber,
            this.MACEconfig.config.MACEComms.ipAddress,
            function() {
                // console.log('Connected to: ' + this.state.tcpHost + ':' + this.state.tcpPort);

                // If vehicle ID == 0, loop over every vehicle ID and make a request:
                if (vehicleID !== 0 || tcpCommand === "GET_CONNECTED_VEHICLES") {
                    let tcpRequest = {
                        tcpCommand: tcpCommand,
                        vehicleID: vehicleID,
                        vehicleCommand: vehicleCommand
                    };
                    socket.write(JSON.stringify(tcpRequest));
                    socket.end();
                } else if (vehicleID === 0) {
                    // for(let vehicle in this.connectedVehicles) {
                    for (let vehicle in this.vehicleDB) {
                        if (vehicle) {
                            let tcpRequest = {
                                tcpCommand: tcpCommand,
                                vehicleID: vehicle,
                                vehicleCommand: vehicleCommand
                            };
                            socket.write(JSON.stringify(tcpRequest));
                            socket.end();
                        }
                    }
                }
            }.bind(this)
        );
    };

    setupTCPClient = (socket: any) => {
        // Add a 'data' event handler for the client socket
        // data is what the server sent to this socket
        socket.on(
            "data",
            function(data: any) {
                // console.log('DATA: ' + data);
                // let jsonData = JSON.parse(data);
                // this.parseTCPServerData(jsonData);
                // Close the client socket completely
                // socket.destroy();
            }.bind(this)
        );

        // Add a 'close' event handler for the client socket
        socket.on(
            "close",
            function() {
                // console.log('Connection closed');
                socket.destroy();
            }.bind(this)
        );

        // Add an 'error' event handler
        socket.on(
            "error",
            function(err: any) {
                console.log("Socket " + err);
                let str = err + "";
                if (str.indexOf("ECONNREFUSED") > 0) {
                    // this.handleClearGUI();
                    // let title = '';
                    // let level = 'error'
                    // this.showNotification(title, 'Lost connection to MACE. ', level, 'tc', 'Got it');
                }
                socket.destroy();
            }.bind(this)
        );
    };
}
