import * as React from "react";
import { store } from "react-notifications-component";
import { Provider } from "./Context";
import {
  areObjectsSame,
  parseJson,
  constructDefaultAircraft
} from "./util/helpers";
import { cloneDeep, forEach } from 'lodash';
import colors from "./util/colors";
const { ipcRenderer } = window.require("electron");
const net = window.require("net");
const path = window.require("path");
const config = require(path.join(
  path.dirname(__dirname),
  "config",
  "config.json"
));
const { map_port } = config;
import PortConfiguration from "./components/configuration/port";
const fs = window.require("fs");


const dgram = window.require("dgram");


import * as Types from "./data-types/index";
import aircraft from "./components/map/components/aircraft";

const DEFAULT_MAP_PORT = 8080;

type Props = {
  location?: any;
};
export type State = {
  boundaries?: Types.Environment.BoundaryPayload[];
  aircrafts?: Types.Aircraft.AircraftPayload[];
  localTargets?: Types.Aircraft.TargetPayload[];
  globalTargets?: Types.Aircraft.TargetPayload[];
  paths?: Types.Aircraft.PathPayload[];
  connectionState?: "connecting" | "connected" | "disconnected";
  icons?: Types.Environment.IconPayload[];
  initialLocation?: Types.Vertex;
  initialZoom?: number;
  messages?: Types.Message[];
  zoom?: number;
  showPortConfiguration?: boolean;
  ports?: { [port_type: string]: number };
};

export default class AppProvider extends React.Component<Props, State> {
  _messageBuffer: Types.Message[] = [];
  _sockets: any[] = [];
  _server: any = null;
  _message: string = "";
  _message_separator: string = "\r";
  _port: number = 8080;
  _aircraft_list: {_aircrafts: Types.Aircraft.AircraftPayload[], last_updated: number} = {_aircrafts: [], last_updated: Date.now()};
  _localTargets_list: {_localTargets: Types.Aircraft.TargetPayload[], last_updated: number} = {_localTargets: [], last_updated: Date.now()};
  _globalTargets_list: {_globalTargets: Types.Aircraft.TargetPayload[], last_updated: number} = {_globalTargets: [], last_updated: Date.now()};
  _paths_list: {_paths: Types.Aircraft.PathPayload[], last_updated: number} = {_paths: [], last_updated: Date.now()};
  _icons_list: {_icons: Types.Environment.IconPayload[], last_updated: number} = {_icons: [], last_updated: Date.now()};
  constructor(props: Props) {
    super(props);
    this.state = {
      boundaries: [],
      aircrafts: [],
      localTargets: [],
      globalTargets: [],
      paths: [],
      icons: [],
      connectionState: "disconnected",
      initialLocation: null,
      initialZoom: 5,
      messages: [],
      zoom: null,
      showPortConfiguration: false,
      ports: {
        map_port: map_port || DEFAULT_MAP_PORT
      }
    };
    // this.spamGUI();

    this.setupUpdateInterval();
  }

    setupUpdateInterval = () => {
        setInterval(() => {
            let update = false;
            let updateObject = {};
            if (!areObjectsSame(this._aircraft_list._aircrafts, this.state.aircrafts)) {
                // this.setState({ aircrafts: this._aircrafts });
                update = true;
                // updateObject = updateObject && {aircrafts: this._aircraft_list._aircrafts};
            }
            if (!areObjectsSame(this._localTargets_list._localTargets, this.state.localTargets)) {
                // this.setState({ localTargets: this._localTargets });
                update = true;
                // updateObject = updateObject && {localTargets: this._localTargets_list._localTargets};
            } 
            if (!areObjectsSame(this._globalTargets_list._globalTargets, this.state.globalTargets)) {
                // this.setState({ globalTargets: this._globalTargets });
                update = true;
                // updateObject = updateObject && {globalTargets: this._globalTargets_list._globalTargets};
            } 
            if (!areObjectsSame(this._paths_list._paths, this.state.paths)) {
                // this.setState({ paths: this._paths });
                update = true;
                // updateObject = updateObject && {paths: this._paths_list._paths};
            }  
            if (!areObjectsSame(this._icons_list._icons, this.state.icons)) {
                // this.setState({ icons: this._icons });
                update = true;
                // updateObject = updateObject && {icons: this._icons_list._icons};
            }

            if(update) {
                this.setState({
                    aircrafts: this._aircraft_list._aircrafts,
                    localTargets: this._localTargets_list._localTargets,
                    globalTargets: this._globalTargets_list._globalTargets,
                    paths: this._paths_list._paths,
                    icons: this._icons_list._icons
                })
            }
        }, 100);
    };

  determinePort = () => {
    switch (window.location.hash) {
      case "#/":
      case "":
        this._port = this.state.ports.map_port;
        break;
      default:
      // Do nothing
    }
  };

  getObjectCount = () => {
    const {
      boundaries,
      aircrafts,
      icons,
      localTargets,
      globalTargets,
      paths
    } = this.state;
    return (
      boundaries.length +
      aircrafts.length +
      icons.length +
      localTargets.length +
      globalTargets.length +
      paths.length
    );
  };

  initServer = () => {
    var PORT = this._port;
    var HOST = '127.0.0.1'; // Not needed...

    this._server = dgram.createSocket('udp4');

    this._server.on('listening', function() {
        console.log('UDP Server listening on ' + HOST + ':' + PORT);
    });

    this._server.on('message', function(message, remote) {
        let messages = parseJson(message.toString());
        messages.forEach((m) => {
            this.onMessage(m as Types.Message);
        });
    }.bind(this));

    this._server.bind(PORT);
  };

  onMessage = (message: Types.Message) => {
    const { messages } = this.state;
    // TODO: Should add this to the other side
    // this.setState({ messages: messages.concat(message) });
    const { message_type, ...rest } = message;
    if (rest.should_display) {
      this.setState({ messages: messages.concat(message) });
    }
    // this._messageBuffer.push(message);
      switch (message_type) {
          case "environment_boundary":
              this.updateBoundaries(rest as Types.Environment.BoundaryPayload);
              break;
          case "environment_icon":
              this.updateIcons(rest as Types.Environment.IconPayload);
              break;
          case "vehicle_heartbeat":
              this.updateAircrafts(rest as Types.Aircraft.HeartbeatPayload);
              break;
          case "vehicle_position":
              this.updateAircraftPosition(rest as Types.Aircraft.PositionPayload);
              break;
          case "vehicle_attitude":
              this.updateAircraftAttitude(rest as Types.Aircraft.AttitudePayload);
              break;
          case "vehicle_airspeed":
              this.updateAircraftAirspeed(rest as Types.Aircraft.AirspeedPayload);
              break;
          case "vehicle_arm":
              this.updateAircraftArmed(rest as Types.Aircraft.ArmPayload);
              break;
          case "vehicle_gps":
              this.updateAircraftGPS(rest as Types.Aircraft.GPSPayload);
              break;
          case "vehicle_text":
              this.updateAircraftText(rest as Types.Aircraft.TextPayload);
              break;
          case "vehicle_mode":
              this.updateAircraftMode(rest as Types.Aircraft.ModePayload);
              break;
          case "vehicle_fuel":
              this.updateAircraftFuel(rest as Types.Aircraft.FuelPayload);
              break;
          case "vehicle_path":
              this.updatePaths(rest as Types.Aircraft.PathPayload);
              break;
          case "vehicle_target":
              this.updateTargets(rest as Types.Aircraft.TargetPayload);
              break;
          case "vehicle_parameter_list":
              this.updateParameters(rest as Types.Aircraft.ParametersPayload);
              break;
          default:
          // do nothing
      }
  };
  sendToHMI = (message: string) => {
    let outgoing = new net.Socket();
    outgoing.connect(8082, `127.0.0.1`, () => {
      outgoing.write(message);
      outgoing.destroy();
    });
    outgoing.on("close", () => {
      outgoing = null;
    });
  };

  componentDidMount() {
    ipcRenderer.on("hmi", (event, message) => {
      this.sendToHMI(JSON.stringify(message));
    });
    ipcRenderer.on("app-close", (event, message) => {
      ipcRenderer.send("closed", {
        messages: JSON.stringify(this._messageBuffer)
      });
    });
    ipcRenderer.on("settings", (event, message) => {
      const { type } = message;
      console.log("type: ", type);
      switch (type) {
        case "Ports":
          this.openPortConfiguration();
          break;
        default: // Do nothing
      }
    });
    ipcRenderer.on("server", (event, message) => {
      try {
        const messages = parseJson(message);
        messages.forEach((m) => {
          this.onMessage(m as Types.Message);
        });
      } catch (error) {
        console.log("Failed to parse message: ", error);
      }
    });
    this.loadPortsFromStorage();
    // Below will add some default UI events, disable this when using the sockets
    // this.addDefaultEvents();
    // this.spamGUI();
  }
  loadPortsFromStorage = () => {
    const { ports } = this.state;
    const savedMapPort = localStorage.getItem("map_port");
    if (savedMapPort && savedMapPort !== "") {
      ports.map_port = parseInt(savedMapPort);
    }
    this.setState({ ports });
    // TODO: Reset the sockets
    this.determinePort();
    this.initServer();
  };
  savePortsInStorage = (ports: { [port_type: string]: number }) => {
    this.setState({ ports });
    localStorage.setItem("map_port", ports.map_port.toString());
    config.map_port = ports.map_port;
    const configString = JSON.stringify(config);
    const configFilePath = path.join(
      path.dirname(__dirname),
      "config",
      "config.json"
    );
    try {
      fs.writeFileSync(configFilePath, configString);
      ipcRenderer.send("reload");
    } catch (error) {
      console.log("Error writing settings file: ", error);
    }
  };
  openPortConfiguration = () => {
    this.setState({ showPortConfiguration: true });
  };
  closePortConfiguration = () => {
    this.setState({ showPortConfiguration: false });
  };
  onMessageString = (message: string) => {
    const messages = parseJson(message);
    messages.forEach((m) => {
      this.onMessage(m as Types.Message);
    });
  };

  addNotification = (notification: Types.Notification) => {
    store.addNotification({
        title: notification.title,
        message: notification.message,
        type: notification.type,
        insert: "top",
        container: "top-center",
        animationIn: ["animate__animated", "animate__fadeIn"],
        animationOut: ["animate__animated", "animate__fadeOut"],
        dismiss: {
          duration: 5000,
          onScreen: true
        }
      });
  };

  // addDefaultEvents = () => {
  //   this.updateBoundaries({
  //     boundary_name: "test",
  //     boundary_type: "hard",
  //     vertices: [
  //       { lat: 38.33640510467016, lng: -76.78361892700197 },
  //       { lat: 38.336135807840115, lng: -76.5630340576172 },
  //       { lat: 38.29465215149741, lng: -76.49368286132814 },
  //       { lat: 38.26082811638795, lng: -76.5571975708008 },
  //       { lat: 38.260288960391925, lng: -76.77658081054689 }
  //     ]
  //   });
  //   this.updateBoundaries({
  //     boundary_name: "test2",
  //     boundary_type: "soft",
  //     vertices: [
  //       { lat: 38.32724845095056, lng: -76.77091598510744 },
  //       { lat: 38.32670978823017, lng: -76.57281875610353 },
  //       { lat: 38.29519105238491, lng: -76.53470993041994 },
  //       { lat: 38.27187993253012, lng: -76.56372070312501 },
  //       { lat: 38.26972361264482, lng: -76.76061630249025 }
  //     ]
  //   });
  //   this.updateAircrafts({
  //     agentID: "agent_805",
  //     vehicle_state: "",
  //     behavior_state: "",
  //     vehicle_type: "QUADROTOR",
  //     autopilot: "",
  //     companion: false,
  //     protocol: "",
  //     mission_state: 0,
  //     mavlink_id: 0,
  //   });
  //   setTimeout(() => {
  //     this.updateAircrafts({
  //       agentID: "agent_124",
  //       vehicle_state: "",
  //       behavior_state: "",
  //       vehicle_type: "QUADROTOR",
  //       autopilot: "",
  //       companion: false,
  //       protocol: "",
  //       mission_state: 0,
  //       mavlink_id: 0,
  //     });
  //   }, 100);
  //   // this.updateTargets({
  //   //   agentID: "agent_123",
  //   //   location: { lat: 38.31903340948611, lng: -76.59444808959962 }
  //   // });
  //   this.updateTargets({
  //     agentID: "agent_805",
  //     location: { lat: 38.31903340948611, lng: -76.59444808959962 }
  //     // is_global: false
  //   });
  //   setTimeout(() => {
  //     this.updateTargets({
  //       agentID: "agent_124",
  //       location: { lat: 38.4, lng: -76.65 }
  //       // is_global: false
  //     });
  //   }, 100);
  //   setTimeout(() => {
  //     this.updateAircrafts({
  //       agentID: "agent_805",
  //       vehicle_state: "",
  //       behavior_state: "",
  //       vehicle_type: "QUADROTOR",
  //       autopilot: "",
  //       companion: false,
  //       protocol: "",
  //       mission_state: 0,
  //       mavlink_id: 0,
  //     });
  //   }, 5000);
    // this.updatePaths({
    //   agentID: "path1",
    //   vertices: [
    //     { lat: 38.28454701883166, lng: -76.66688919067384 },
    //     { lat: 38.28872397758921, lng: -76.66259765625 },
    //     { lat: 38.29343960986726, lng: -76.65264129638673 },
    //     { lat: 38.29748133650612, lng: -76.64594650268556 },
    //     { lat: 38.305833524797066, lng: -76.64302825927736 },
    //     { lat: 38.30879697336429, lng: -76.63101196289064 },
    //     { lat: 38.30798877212676, lng: -76.62139892578126 },
    //     { lat: 38.305698819712546, lng: -76.61487579345705 },
    //     { lat: 38.30771936971291, lng: -76.60526275634767 },
    //     { lat: 38.319302769869594, lng: -76.59444808959962 }
    //   ],
    // });
    // this.simulatePath([
    //   { lat: 38.28454701883166, lng: -76.66688919067384 },
    //   { lat: 38.28872397758921, lng: -76.66259765625 },
    //   { lat: 38.29343960986726, lng: -76.65264129638673 },
    //   { lat: 38.29748133650612, lng: -76.64594650268556 },
    //   { lat: 38.305833524797066, lng: -76.64302825927736 },
    //   { lat: 38.30879697336429, lng: -76.63101196289064 },
    //   { lat: 38.30798877212676, lng: -76.62139892578126 },
    //   { lat: 38.305698819712546, lng: -76.61487579345705 },
    //   { lat: 38.30771936971291, lng: -76.60526275634767 },
    //   { lat: 38.319302769869594, lng: -76.59444808959962 }
    // ]);
    // this.updateIcons({
    //   type: "takeoff_land",
    //   lat: 38.25193153082154, lng: -76.71855926513673,
    //   name: "test-home"
    // });
    // this.updateIcons({
    //   type: "mission_target",
    //   lat: 38.283738547437515, lng: -76.73744201660158,
    //   name: "test-mission-target"
    // });
    // this.updateIcons({
    //   type: "origin",
    //   lat: 38.312568460056966, lng: -76.69469833374025,
    //   name: "test-origin"
    // });
    // this.updateIcons({
    //   type: "command_control",
    //   lat: 38.28158257969907, lng: -76.61453247070314,
    //   name: "test-origin"
    // });

  // };
  // simulatePath = (path: Vertices) => {
  //   let index = 0;
  //   setInterval(() => {
  //     if (index <= path.length - 1) {
  //       this.updateAircrafts({
  //         agentID: "agent_123",
  //         vehicle_state: "",
  //         behavior_state: "",
  //         vehicle_type: "QUADROTOR",
  //         autopilot: "",
  //         companion: false,
  //         protocol: "",
  //         mission_state: 0,
  //         mavlink_id: 0,
  //       });
  //       index++;
  //     }
  //   }, 1000);
  // };
  updateBoundaries = (boundary: Types.Environment.BoundaryPayload) => {
    const { ...all } = boundary;
    let boundaries = [...this.state.boundaries];
    const existingIndex = boundaries.findIndex(
      (b) => boundary.boundary_name === b.boundary_name
    );
    if (existingIndex !== -1) {
      // if (!areObjectsSame(boundaries[existingIndex], { ...all })) {
      boundaries[existingIndex] = { ...all };
      // }
    } else {
      boundaries.push({ ...all });
    }
    if (!areObjectsSame(boundaries, this.state.boundaries)) {
      this.setState({ boundaries });
    }
  };
  updateAircrafts = (heartbeat: Types.Aircraft.HeartbeatPayload) => {
        const { ...all } = location;
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => heartbeat.agentID === a.agentID
        );

        if (existingIndex !== -1) {
            // aircrafts[existingIndex].autopilot = heartbeat.autopilot;
            aircrafts[existingIndex].vehicle_type = heartbeat.vehicle_type;
            // aircrafts[existingIndex].companion = heartbeat.companion;
            // aircrafts[existingIndex].protocol = heartbeat.protocol;
            // aircrafts[existingIndex].mission_state = heartbeat.mission_state;
            // aircrafts[existingIndex].mavlink_id = heartbeat.mavlink_id;
            aircrafts[existingIndex].behavior_state = heartbeat.behavior_state;
            aircrafts[existingIndex].vehicle_state = heartbeat.vehicle_state;
            aircrafts[existingIndex].date = heartbeat.date;
            aircrafts[existingIndex].should_display = heartbeat.should_display;
            // aircrafts[existingIndex].lastUpdate = heartbeat.lastUpdate;
            aircrafts[existingIndex].lastUpdate = Date.now();
        } else {
            let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length;);
            tmpAC.agentID = heartbeat.agentID;
            // tmpAC.autopilot = heartbeat.autopilot;
            tmpAC.vehicle_type = heartbeat.vehicle_type;
            // tmpAC.companion = heartbeat.companion;
            // tmpAC.protocol = heartbeat.protocol;
            // tmpAC.mission_state = heartbeat.mission_state;
            // tmpAC.mavlink_id = heartbeat.mavlink_id;
            tmpAC.behavior_state = heartbeat.behavior_state;
            tmpAC.vehicle_state = heartbeat.vehicle_state;
            tmpAC.date = heartbeat.date;
            tmpAC.should_display = heartbeat.should_display;
            // tmpAC.lastUpdate = heartbeat.lastUpdate;
            tmpAC.lastUpdate = Date.now();
            aircrafts.push(tmpAC);
        }

        // Sort array based on Agent ID (assumes agentID is a stringified integer)
        aircrafts.sort(function(a, b) {
            return (parseInt(a.agentID) - parseInt(b.agentID));
        });

        if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
            this._aircraft_list._aircrafts = aircrafts;
            this._aircraft_list.last_updated = Date.now();
        }
  };
//   pickAircraftColor = (): Types.ColorObject => {
//     let numAircraft = this._aircraft_list._aircrafts.length;
//     while (numAircraft > 9) {
//       numAircraft-=10;
//     }
//     switch (numAircraft) {
//       case 0:
//         return colors.blue;
//       case 1:
//         return colors.green;
//       case 2:
//         return colors.pink;
//       case 3:
//         return colors.orange;
//       case 4:
//         return colors.purple;
//       case 5:
//         return colors.red;
//       case 6:
//         return colors.yellow;
//       case 7:
//         return colors.teal;
//       case 8:
//         return colors.indigo;
//       case 9:
//         return colors.gray;
//     }
//   }

//   constructDefaultAircraft = (): Types.Aircraft.AircraftPayload => {
//       // TODO-PAT: Commented out for spamming testing:
//     // this.sendToMACE("GET_ENVIRONMENT_BOUNDARY",[],[]);

//     let now = new Date();
//     let timestamp = now.getTime();
//     return {
//         agentID: "DEFAULT",
//         selected: true,
//         behavior_state: "",
//         vehicle_state: "UNINITIALIZED",
//         vehicle_type: "QUADROTOR",
//         color: this.pickAircraftColor(),
//         orientation: {
//             pitch: 0.0,
//             roll: 0.0,
//             yaw: 0.0
//         },
//         location: {
//             lat: 0.0,
//             lng: 0.0,
//             alt: 0.0
//         },
//         armed: false,
//         visible_sats: 0.0,
//         gps_fix: "",
//         hdop: 0.0,
//         vdop: 0.0,
//         text: {
//             textStr: "No messages",
//             textSeverity: "",
//             textTimestamp: timestamp
//         },
//         mode: "",
//         battery_remaining: 0.0,
//         battery_current: 0.0,
//         battery_voltage: 0.0,
//         param_list: [
//             {
//                 param_id: "TKOFF_ALT",
//                 value: 50
//             }
//         ],
//         airspeed: 0.0,
//         distance_to_target: 0.0,
//         flight_time: 0.0,
//         lastUpdate: Date.now()
//     }
//   }

    updateAircraftPosition = (position: Types.Aircraft.PositionPayload) => {
        const { ...all } = location;
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => position.agentID === a.agentID
        );

        if (existingIndex !== -1) {
            aircrafts[existingIndex].location.lat = position.lat;
            aircrafts[existingIndex].location.lng = position.lng;
            if(position.alt) {
                aircrafts[existingIndex].location.alt = position.alt;
            }
            aircrafts[existingIndex].lastUpdate = Date.now();
        } else {
            let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length);
            tmpAC.agentID = position.agentID;
            tmpAC.location.lat = position.lat;
            tmpAC.location.lng = position.lng;
            tmpAC.location.alt = position.alt ? position.alt : 0.0;
            tmpAC.lastUpdate = Date.now();
            aircrafts.push(tmpAC);
        }

        if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
            this._aircraft_list._aircrafts = aircrafts;
            this._aircraft_list.last_updated = Date.now();
        }
    }

    updateAircraftAttitude = (attitude: Types.Aircraft.AttitudePayload) => {
        const { ...all } = attitude;
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => attitude.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].orientation.roll = attitude.roll;
            aircrafts[existingIndex].orientation.pitch = attitude.pitch;
            aircrafts[existingIndex].orientation.yaw = attitude.yaw;
            aircrafts[existingIndex].lastUpdate = Date.now();
        } else {
            let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length);
            tmpAC.agentID = attitude.agentID;
            tmpAC.orientation.roll = attitude.roll;
            tmpAC.orientation.pitch = attitude.pitch;
            tmpAC.orientation.yaw = attitude.yaw;
            tmpAC.lastUpdate = Date.now();
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
            this._aircraft_list._aircrafts = aircrafts;
            this._aircraft_list.last_updated = Date.now();
        }
    }

    updateAircraftAirspeed = (airspeed: Types.Aircraft.AirspeedPayload) => {
        const { ...all } = airspeed;
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => airspeed.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].airspeed = airspeed.airspeed;
            aircrafts[existingIndex].lastUpdate = Date.now();
        } else {
            let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length);
            tmpAC.agentID = airspeed.agentID;
            tmpAC.airspeed = airspeed.airspeed;
            tmpAC.lastUpdate = Date.now();
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
            this._aircraft_list._aircrafts = aircrafts;
            this._aircraft_list.last_updated = Date.now();
        }
    }

    updateAircraftArmed = (armed: Types.Aircraft.ArmPayload) => {
        const { ...all } = armed;
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => armed.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].armed = armed.armed;
            aircrafts[existingIndex].lastUpdate = Date.now();
        } else {
            let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length);
            tmpAC.agentID = armed.agentID;
            tmpAC.armed = armed.armed;
            tmpAC.lastUpdate = Date.now();
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
            this._aircraft_list._aircrafts = aircrafts;
            this._aircraft_list.last_updated = Date.now();
        }
    }

    updateAircraftGPS = (gps: Types.Aircraft.GPSPayload) => {
        const { ...all } = gps;
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => gps.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].gps_fix = gps.gps_fix;
            aircrafts[existingIndex].vdop = gps.vdop;
            aircrafts[existingIndex].hdop = gps.hdop;
            aircrafts[existingIndex].visible_sats = gps.visible_sats;
            aircrafts[existingIndex].lastUpdate = Date.now();
        } else {
            let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length);
            tmpAC.agentID = gps.agentID;
            tmpAC.gps_fix = gps.gps_fix;
            tmpAC.vdop = gps.vdop;
            tmpAC.hdop = gps.hdop;
            tmpAC.visible_sats = gps.visible_sats;
            tmpAC.lastUpdate = Date.now();
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
            this._aircraft_list._aircrafts = aircrafts;
            this._aircraft_list.last_updated = Date.now();
        }
    }

    updateAircraftText = (text: Types.Aircraft.TextPayload) => {
        const { ...all } = text;
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => text.agentID === a.agentID
        );
        let now = new Date();
        if (existingIndex !== -1) {

            if ((aircrafts[existingIndex].text.textStr !== text.text) && (text.text === "Flight plan received")){
                let command: string = "FORCE_DATA_SYNC";
                let payload = [];
                this.sendToMACE(command, [aircrafts[existingIndex]], payload);
            }

            aircrafts[existingIndex].text = {
                textStr: text.text,
                textSeverity: text.severity,
                textTimestamp: now.getTime()
            };
            aircrafts[existingIndex].lastUpdate = Date.now();
        } else {
            let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length);
            tmpAC.agentID = text.agentID;
            tmpAC.text = {
                textStr: text.text,
                textSeverity: text.severity,
                textTimestamp: now.getTime()
            };
            tmpAC.lastUpdate = Date.now();
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
            this._aircraft_list._aircrafts = aircrafts;
            this._aircraft_list.last_updated = Date.now();
        }
    }

    updateAircraftMode = (mode: Types.Aircraft.ModePayload) => {
        const { ...all } = mode;
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => mode.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].mode = mode.mode;
            aircrafts[existingIndex].lastUpdate = Date.now();
        } else {
            let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length);
            tmpAC.agentID = mode.agentID;
            tmpAC.mode = mode.mode;
            tmpAC.lastUpdate = Date.now();
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
            this._aircraft_list._aircrafts = aircrafts;
            this._aircraft_list.last_updated = Date.now();
        }
    }

    updateAircraftFuel = (fuel: Types.Aircraft.FuelPayload) => {
        const { ...all } = fuel;
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => fuel.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].battery_current = fuel.battery_current;
            aircrafts[existingIndex].battery_remaining = fuel.battery_remaining;
            aircrafts[existingIndex].battery_voltage = fuel.battery_voltage;
            aircrafts[existingIndex].lastUpdate = Date.now();
        } else {
            let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length);
            tmpAC.agentID = fuel.agentID;
            tmpAC.battery_current = fuel.battery_current;
            tmpAC.battery_remaining = fuel.battery_remaining;
            tmpAC.battery_voltage = fuel.battery_voltage;
            tmpAC.lastUpdate = Date.now();
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
            this._aircraft_list._aircrafts = aircrafts;
            this._aircraft_list.last_updated = Date.now();
        }
    }

  updateTargets = (target: Types.Aircraft.TargetPayload) => {
    if (target.is_global) {
      this.updateGlobalTargets(target);
    } else {
        this.updateLocalTargets(target);

        // Update aircraft distance to target:
        let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
        const acIndex = aircrafts.findIndex(
            (a) => target.agentID === a.agentID
        );
        if (acIndex !== -1) {
            aircrafts[acIndex].distance_to_target = target.distance_to_target;
            if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
                this._aircraft_list._aircrafts = aircrafts;
                this._aircraft_list.last_updated = Date.now();
            }
        }
    }
  };

  updateLocalTargets = (target: Types.Aircraft.TargetPayload) => {
    const { ...all } = target;
    let targets = [...this._localTargets_list._localTargets];
    const existingIndex = targets.findIndex(
      (t) => target.agentID === t.agentID
    );
    if (existingIndex !== -1) {
      if (!areObjectsSame(targets[existingIndex], { ...all })) {
        targets[existingIndex] = { ...all };
        targets[existingIndex].lastUpdate = Date.now();
      }
    } else {
        target.lastUpdate = Date.now();
      targets.push(target);
    }
    if (!areObjectsSame(targets, this._localTargets_list._localTargets)) {
        this._localTargets_list._localTargets = targets;
        this._localTargets_list.last_updated = Date.now();
    }   
  };

  updateGlobalTargets = (target: Types.Aircraft.TargetPayload) => {
    const { ...all } = target;
    let targets = [...this._globalTargets_list._globalTargets];
    const existingIndex = targets.findIndex(
      (t) => target.agentID === t.agentID
    );
    if (existingIndex !== -1) {
      if (!areObjectsSame(targets[existingIndex], { ...all })) {
        targets[existingIndex] = { ...all };
        targets[existingIndex].lastUpdate = Date.now();
      }
    } else {
        target.lastUpdate = Date.now();
      targets.push(target);
    }
    if (!areObjectsSame(targets, this._globalTargets_list._globalTargets)) {
        this._globalTargets_list._globalTargets = targets;
        this._globalTargets_list.last_updated = Date.now();
    } 
  };

  updateParameters = (params: Types.Aircraft.ParametersPayload) => {
    const { ...all } = params;
    let aircrafts = cloneDeep(this._aircraft_list._aircrafts);
    const existingIndex = aircrafts.findIndex(
      (a) => params.agentID === a.agentID
    );
    if (existingIndex !== -1) {
        // aircrafts[existingIndex].battery_current = fuel.battery_current;
    } else {
        let tmpAC: Types.Aircraft.AircraftPayload = constructDefaultAircraft(this._aircraft_list._aircrafts.length);
        tmpAC.agentID = params.agentID;
        tmpAC.param_list = params.param_list;
        tmpAC.lastUpdate = Date.now();
        aircrafts.push(tmpAC);
    }
    if (!areObjectsSame(aircrafts, this._aircraft_list._aircrafts)) {
        this._aircraft_list._aircrafts = aircrafts;
        this._aircraft_list.last_updated = Date.now();
    } 
  }

  updatePaths = (path: Types.Aircraft.PathPayload) => {
    const { ...all } = path;
    let paths = [...this._paths_list._paths];
    const existingIndex = paths.findIndex((p) => path.agentID === p.agentID);
    if (existingIndex !== -1) {
      if (!areObjectsSame(paths[existingIndex], { ...all })) {
        paths[existingIndex] = { ...all };
        paths[existingIndex].lastUpdate = Date.now();
      }
    } else {
        path.lastUpdate = Date.now();
        paths.push(path);
    }
    if (!areObjectsSame(paths, this._paths_list._paths)) {
        this._paths_list._paths = paths;
        this._paths_list.last_updated = Date.now();
    }
  };
  updateIcons = (icon: Types.Environment.IconPayload) => {
    const { ...all } = icon;
    const icons = [...this._icons_list._icons];
    const existingIndex = icons.findIndex((i) => icon.name === i.name);
    if (existingIndex !== -1) {
      if (!areObjectsSame(icons[existingIndex], { ...all })) {
        icons[existingIndex] = { ...all };
        icons[existingIndex].lastUpdate = Date.now();
      }
    } else {
        icon.lastUpdate = Date.now();
        icons.push(icon);
    }
    if (!areObjectsSame(icons, this._icons_list._icons)) {
        this._icons_list._icons = icons;
        this._icons_list.last_updated = Date.now();
    }
  };
  removeIcon = (name: string) => {
    const { icons } = this.state;
    const existingIndex = icons.findIndex((i) => i.name === name);
    if (existingIndex !== -1) {
      icons.splice(existingIndex, 1);
    }
    this.setState({ icons });
  };
  setZoom = (zoom: number) => {
    this.setState({ zoom });
  };

  

  sendToMACE = (command: string, filteredAircrafts: Types.Aircraft.AircraftPayload[], payload: string[]) => {
      let aircraftIds = [];
      filteredAircrafts.forEach((a) => {
          aircraftIds.push(a.agentID);
      });
      let maceCmd = {
          command: command,
          aircraft: aircraftIds,
          data: payload
      };

      console.log(JSON.stringify(maceCmd));

      let outgoing = new net.Socket();
      outgoing.connect(8082, `127.0.0.1`, () => {
        outgoing.write(JSON.stringify(maceCmd));
        outgoing.destroy();
      });
      outgoing.on("close", () => {
        outgoing = null;
      });
  }

    updateSelectedAircraft = (agentIDs: string[], show?: boolean) => {
        let aircrafts = cloneDeep(this.state.aircrafts);
        agentIDs.forEach(id => {
            const existingIndex = aircrafts.findIndex(
                (a) => id === a.agentID
              );
              if (existingIndex !== -1) {
                  if(show === undefined) {
                      aircrafts[existingIndex].selected = !aircrafts[existingIndex].selected;
                  }
                  else {
                      aircrafts[existingIndex].selected = show;
                  }
              }
        });

        if (!areObjectsSame(aircrafts, this.state.aircrafts)) {
          this.setState({ aircrafts });
        }
    }

  render() {
    return (
      <Provider
        value={{
          ...this.state,
          updateIcons: this.updateIcons,
          updateTargets: this.updateTargets,
          removeIcon: this.removeIcon,
          setGlobalZoom: this.setZoom,
          sendToMACE: this.sendToMACE,
          updateSelectedAircraft: this.updateSelectedAircraft,
          addNotification: this.addNotification
        }}
      >
        <>
          {this.props.children}
          <PortConfiguration
            open={this.state.showPortConfiguration}
            onRequestClose={this.closePortConfiguration}
            onSave={this.savePortsInStorage}
            defaultPorts={this.state.ports}
          />
        </>
      </Provider>
    );
  }
}
