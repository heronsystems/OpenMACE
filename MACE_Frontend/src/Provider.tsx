import * as React from "react";
import { store } from "react-notifications-component";
import { Provider } from "./Context";
import {
  areObjectsSame,
  parseJson
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

const DEFAULT_MAP_PORT = 8080;

type Props = {
  location?: any;
};
export type State = {
  boundaries?: Environment.BoundaryPayload[];
  aircrafts?: Aircraft.AircraftPayload[];
  localTargets?: Aircraft.TargetPayload[];
  globalTargets?: Aircraft.TargetPayload[];
  paths?: Aircraft.PathPayload[];
  connectionState?: "connecting" | "connected" | "disconnected";
  icons?: Environment.IconPayload[];
  initialLocation?: Vertex;
  initialZoom?: number;
  messages?: Message[];
  zoom?: number;
  showPortConfiguration?: boolean;
  ports?: { [port_type: string]: number };
};

export default class AppProvider extends React.Component<Props, State> {
  _messageBuffer: Message[] = [];
  _sockets: any[] = [];
  _server: any = null;
  _message: string = "";
  _message_separator: string = "\r";
  _port: number = 8080;
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
  }
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
    // Avoid dead sockets by responding to the 'end' event
    //   const sockets = [];
    //   var message = ""; // variable that collects chunks
    //   var message_separater = "\r";
    //   this.sockets = [];
    //   this.message = "";
    //   this.message_separator = "\r";
    // Create a TCP socket listener
    this._server = net.Server(
      function (socket) {
        // Add the new client socket connection to the array of
        // sockets
        this._message = "";
        this._sockets.push(socket);
        // console.log("Added new socket", this._sockets.length);
        // 'data' is an event that means that a message was just sent by the
        // client application
        socket.on("data", (data) => {
          //   console.log("Open socket connections: ", this._sockets.length);

          this._message += data;

          let message_separator_index = this._message.indexOf(
            this._message_separator
          );
          let foundEntireMessage = message_separator_index !== -1;

        //   console.log(foundEntireMessage);
        //   if (foundEntireMessage) {
            let msg = this._message.slice(0, message_separator_index);

            let messages = parseJson(msg);

            // console.log(messages);
            messages.forEach((m) => {
              this.onMessage(m as Message);
            });

            // Respond to client for proper closing:
            msg += this._message_separator;
            // this._sockets.forEach((s) => {
            //   s.write("GUI received message, closing socket.\r");
            // });

            this._message = this._message.slice(message_separator_index + 1);
        //   } else {
        //     let messages = parseJson(this._message);

        //     // console.log(messages);
        //   }
        });

        // Use splice to get rid of the socket that is ending.
        // The 'end' event means tcp client has disconnected.
        socket.on("end", () => {
          // console.log("socket done");
          // const i = sockets.indexOf(socket);
          // sockets.splice(i, 1);
        });
        socket.on("close", () => {
          // console.log("Trying to close");
          const i = this._sockets.indexOf(socket);
          this._sockets.splice(i, 1);
        });
        socket.on("error", (err) => {
          console.log("Error with socket: ", err);
        });
      }.bind(this)
    );
    this._server.listen(this._port);
    console.log("System waiting at http://localhost:8080");
  };

  onMessage = (message: Message) => {
    const { messages } = this.state;
    // TODO: Should add this to the other side
    // this.setState({ messages: messages.concat(message) });
    const { message_type, ...rest } = message;
    if (rest.should_display) {
      this.setState({ messages: messages.concat(message) });
    }
    this._messageBuffer.push(message);
      switch (message_type) {
          case "environment_boundary":
              this.updateBoundaries(rest as Environment.BoundaryPayload);
              break;
          case "environment_icon":
              this.updateIcons(rest as Environment.IconPayload);
              break;
          case "vehicle_heartbeat":
              this.updateAircrafts(rest as Aircraft.HeartbeatPayload);
              break;
          case "vehicle_position":
              this.updateAircraftPosition(rest as Aircraft.PositionPayload);
              break;
          case "vehicle_attitude":
              this.updateAircraftAttitude(rest as Aircraft.AttitudePayload);
              break;
          case "vehicle_arm":
              this.updateAircraftArmed(rest as Aircraft.ArmPayload);
              break;
          case "vehicle_gps":
              this.updateAircraftGPS(rest as Aircraft.GPSPayload);
              break;
          case "vehicle_text":
              this.updateAircraftText(rest as Aircraft.TextPayload);
              break;
          case "vehicle_mode":
              this.updateAircraftMode(rest as Aircraft.ModePayload);
              break;
          case "vehicle_fuel":
              this.updateAircraftFuel(rest as Aircraft.FuelPayload);
              break;
          case "vehicle_path":
              this.updatePaths(rest as Aircraft.PathPayload);
              break;
          case "vehicle_target":
              this.updateTargets(rest as Aircraft.TargetPayload);
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
          this.onMessage(m as Message);
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
      this.onMessage(m as Message);
    });
  };

  addNotification = ({
    title,
    message,
    type
  }: {
    title: string;
    message: string;
    type: "error" | "success" | "info" | "warning";
  }) => {
    store.addNotification({
      title,
      message,
      type,
      container: "top-right", // where to position the notifications
      animationIn: ["animated", "fadeIn"], // animate.css classes that's applied
      animationOut: ["animated", "fadeOut"], // animate.css classes that's applied
      dismiss: {
        duration: 3000
      }
    });
  };
  addDefaultEvents = () => {
    this.updateBoundaries({
      boundary_name: "test",
      boundary_type: "hard",
      vertices: [
        { lat: 38.33640510467016, lng: -76.78361892700197 },
        { lat: 38.336135807840115, lng: -76.5630340576172 },
        { lat: 38.29465215149741, lng: -76.49368286132814 },
        { lat: 38.26082811638795, lng: -76.5571975708008 },
        { lat: 38.260288960391925, lng: -76.77658081054689 }
      ]
    });
    this.updateBoundaries({
      boundary_name: "test2",
      boundary_type: "soft",
      vertices: [
        { lat: 38.32724845095056, lng: -76.77091598510744 },
        { lat: 38.32670978823017, lng: -76.57281875610353 },
        { lat: 38.29519105238491, lng: -76.53470993041994 },
        { lat: 38.27187993253012, lng: -76.56372070312501 },
        { lat: 38.26972361264482, lng: -76.76061630249025 }
      ]
    });
    this.updateAircrafts({
      agentID: "agent_805",
      vehicle_state: "",
      behavior_state: "",
      vehicle_type: "QUADROTOR",
      autopilot: "",
      companion: false,
      protocol: "",
      mission_state: 0,
      mavlink_id: 0,
    });
    setTimeout(() => {
      this.updateAircrafts({
        agentID: "agent_124",
        vehicle_state: "",
        behavior_state: "",
        vehicle_type: "QUADROTOR",
        autopilot: "",
        companion: false,
        protocol: "",
        mission_state: 0,
        mavlink_id: 0,
      });
    }, 100);
    // this.updateTargets({
    //   agentID: "agent_123",
    //   location: { lat: 38.31903340948611, lng: -76.59444808959962 }
    // });
    this.updateTargets({
      agentID: "agent_805",
      location: { lat: 38.31903340948611, lng: -76.59444808959962 }
      // is_global: false
    });
    setTimeout(() => {
      this.updateTargets({
        agentID: "agent_124",
        location: { lat: 38.4, lng: -76.65 }
        // is_global: false
      });
    }, 100);
    setTimeout(() => {
      this.updateAircrafts({
        agentID: "agent_805",
        vehicle_state: "",
        behavior_state: "",
        vehicle_type: "QUADROTOR",
        autopilot: "",
        companion: false,
        protocol: "",
        mission_state: 0,
        mavlink_id: 0,
      });
    }, 5000);
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
    this.updateIcons({
      type: "takeoff_land",
      lat: 38.25193153082154, lng: -76.71855926513673,
      name: "test-home"
    });
    this.updateIcons({
      type: "mission_target",
      lat: 38.283738547437515, lng: -76.73744201660158,
      name: "test-mission-target"
    });
    this.updateIcons({
      type: "origin",
      lat: 38.312568460056966, lng: -76.69469833374025,
      name: "test-origin"
    });
    this.updateIcons({
      type: "command_control",
      lat: 38.28158257969907, lng: -76.61453247070314,
      name: "test-origin"
    });

  };
  simulatePath = (path: Vertices) => {
    let index = 0;
    setInterval(() => {
      if (index <= path.length - 1) {
        this.updateAircrafts({
          agentID: "agent_123",
          vehicle_state: "",
          behavior_state: "",
          vehicle_type: "QUADROTOR",
          autopilot: "",
          companion: false,
          protocol: "",
          mission_state: 0,
          mavlink_id: 0,
        });
        index++;
      }
    }, 1000);
  };
  updateBoundaries = (boundary: Environment.BoundaryPayload) => {
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
  updateAircrafts = (heartbeat: Aircraft.HeartbeatPayload) => {
    const { ...all } = location;
        let aircrafts = cloneDeep(this.state.aircrafts);
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
            aircrafts[existingIndex].lastUpdate = heartbeat.lastUpdate;
        } else {
            let tmpAC: Aircraft.AircraftPayload = this.constructDefaultAircraft();
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
            tmpAC.lastUpdate = heartbeat.lastUpdate;
            aircrafts.push(tmpAC);
        }

        if (!areObjectsSame(aircrafts, this.state.aircrafts)) {
          this.setState({ aircrafts });
        }
  };
  pickAircraftColor = (): ColorObject => {
    let numAircraft = this.state.aircrafts.length;
    while (numAircraft > 9) {
      numAircraft-=10;
    }
    switch (numAircraft) {
      case 0:
        return colors.blue;
      case 1:
        return colors.green;
      case 2:
        return colors.pink;
      case 3:
        return colors.orange;
      case 4:
        return colors.purple;
      case 5:
        return colors.red;
      case 6:
        return colors.yellow;
      case 7:
        return colors.teal;
      case 8:
        return colors.indigo;
      case 9:
        return colors.gray;
    }
  }

  constructDefaultAircraft = (): Aircraft.AircraftPayload => {
    this.sendToMACE("GET_ENVIRONMENT_BOUNDARY",[],[]);
    return {
        agentID: "DEFAULT",
        selected: true,
        behavior_state: "",
        vehicle_state: "UNINITIALIZED",
        vehicle_type: "QUADROTOR",
        color: this.pickAircraftColor(),
        orientation: {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0
        },
        location: {
            lat: 0.0,
            lng: 0.0,
            alt: 0.0
        },
        armed: false,
        visible_sats: 0.0,
        gps_fix: "",
        hdop: 0.0,
        vdop: 0.0,
        text: {
            textStr: "No messages",
            textSeverity: ""
        },
        mode: "",
        battery_remaining: 0.0,
        battery_current: 0.0,
        battery_voltage: 0.0
    }
  }

    updateAircraftPosition = (position: Aircraft.PositionPayload) => {
        const { ...all } = location;
        let aircrafts = cloneDeep(this.state.aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => position.agentID === a.agentID
        );

        if (existingIndex !== -1) {
            aircrafts[existingIndex].location.lat = position.lat;
            aircrafts[existingIndex].location.lng = position.lng;
            if(position.alt) {
                aircrafts[existingIndex].location.alt = position.alt;
            }
        } else {
            let tmpAC: Aircraft.AircraftPayload = this.constructDefaultAircraft();
            tmpAC.agentID = position.agentID;
            tmpAC.location.lat = position.lat;
            tmpAC.location.lng = position.lng;
            tmpAC.location.alt = position.alt ? position.alt : 0.0;
            aircrafts.push(tmpAC);
        }

        if (!areObjectsSame(aircrafts, this.state.aircrafts)) {
          this.setState({ aircrafts });
        }
    }

    updateAircraftAttitude = (attitude: Aircraft.AttitudePayload) => {
        const { ...all } = attitude;
        let aircrafts = cloneDeep(this.state.aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => attitude.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].orientation.roll = attitude.roll;
            aircrafts[existingIndex].orientation.pitch = attitude.pitch;
            aircrafts[existingIndex].orientation.yaw = attitude.yaw;
        } else {
            let tmpAC: Aircraft.AircraftPayload = this.constructDefaultAircraft();
            tmpAC.agentID = attitude.agentID;
            tmpAC.orientation.roll = attitude.roll;
            tmpAC.orientation.pitch = attitude.pitch;
            tmpAC.orientation.yaw = attitude.yaw;
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this.state.aircrafts)) {
          this.setState({ aircrafts });
        }
    }

    updateAircraftArmed = (armed: Aircraft.ArmPayload) => {
        const { ...all } = armed;
        let aircrafts = cloneDeep(this.state.aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => armed.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].armed = armed.armed;
        } else {
            let tmpAC: Aircraft.AircraftPayload = this.constructDefaultAircraft();
            tmpAC.agentID = armed.agentID;
            tmpAC.armed = armed.armed;
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this.state.aircrafts)) {
          this.setState({ aircrafts });
        }
    }

    updateAircraftGPS = (gps: Aircraft.GPSPayload) => {
        const { ...all } = gps;
        let aircrafts = cloneDeep(this.state.aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => gps.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].gps_fix = gps.gps_fix;
            aircrafts[existingIndex].vdop = gps.vdop;
            aircrafts[existingIndex].hdop = gps.hdop;
            aircrafts[existingIndex].visible_sats = gps.visible_sats;
        } else {
            let tmpAC: Aircraft.AircraftPayload = this.constructDefaultAircraft();
            tmpAC.agentID = gps.agentID;
            tmpAC.gps_fix = gps.gps_fix;
            tmpAC.vdop = gps.vdop;
            tmpAC.hdop = gps.hdop;
            tmpAC.visible_sats = gps.visible_sats;
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this.state.aircrafts)) {
          this.setState({ aircrafts });
        }
    }

    updateAircraftText = (text: Aircraft.TextPayload) => {
        const { ...all } = text;
        let aircrafts = cloneDeep(this.state.aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => text.agentID === a.agentID
        );
        if (existingIndex !== -1) {

            if ((aircrafts[existingIndex].text.textStr !== text.text) && (text.text === "Flight plan received")){
                let command: string = "FORCE_DATA_SYNC";
                let payload = [];
                this.sendToMACE(command, [aircrafts[existingIndex]], payload);
            }

            aircrafts[existingIndex].text = {
                textStr: text.text,
                textSeverity: text.severity
            };
        } else {
            let tmpAC: Aircraft.AircraftPayload = this.constructDefaultAircraft();
            tmpAC.agentID = text.agentID;
            tmpAC.text = {
                textStr: text.text,
                textSeverity: text.severity
            };
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this.state.aircrafts)) {
          this.setState({ aircrafts });
        }
    }

    updateAircraftMode = (mode: Aircraft.ModePayload) => {
        const { ...all } = mode;
        let aircrafts = cloneDeep(this.state.aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => mode.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].mode = mode.mode;
        } else {
            let tmpAC: Aircraft.AircraftPayload = this.constructDefaultAircraft();
            tmpAC.agentID = mode.agentID;
            tmpAC.mode = mode.mode;
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this.state.aircrafts)) {
          this.setState({ aircrafts });
        }
    }

    updateAircraftFuel = (fuel: Aircraft.FuelPayload) => {
        const { ...all } = fuel;
        let aircrafts = cloneDeep(this.state.aircrafts);
        const existingIndex = aircrafts.findIndex(
          (a) => fuel.agentID === a.agentID
        );
        if (existingIndex !== -1) {
            aircrafts[existingIndex].battery_current = fuel.battery_current;
            aircrafts[existingIndex].battery_remaining = fuel.battery_remaining;
            aircrafts[existingIndex].battery_voltage = fuel.battery_voltage;
        } else {
            let tmpAC: Aircraft.AircraftPayload = this.constructDefaultAircraft();
            tmpAC.agentID = fuel.agentID;
            tmpAC.battery_current = fuel.battery_current;
            tmpAC.battery_remaining = fuel.battery_remaining;
            tmpAC.battery_voltage = fuel.battery_voltage;
            aircrafts.push(tmpAC);
        }
        if (!areObjectsSame(aircrafts, this.state.aircrafts)) {
          this.setState({ aircrafts });
        }
    }

  updateTargets = (target: Aircraft.TargetPayload) => {
    if (target.is_global) {
      this.updateGlobalTargets(target);
    } else {
      this.updateLocalTargets(target);
    }
  };

  updateLocalTargets = (target: Aircraft.TargetPayload) => {
    const { ...all } = target;
    let targets = [...this.state.localTargets];
    const existingIndex = targets.findIndex(
      (t) => target.agentID === t.agentID
    );
    if (existingIndex !== -1) {
      if (!areObjectsSame(targets[existingIndex], { ...all })) {
        targets[existingIndex] = { ...all };
      }
    } else {
      targets.push({ ...all });
    }
    if (!areObjectsSame(targets, this.state.localTargets)) {
      this.setState({ localTargets: targets });
    }
  };

  updateGlobalTargets = (target: Aircraft.TargetPayload) => {
    const { ...all } = target;
    let targets = [...this.state.globalTargets];
    const existingIndex = targets.findIndex(
      (t) => target.agentID === t.agentID
    );
    if (existingIndex !== -1) {
      if (!areObjectsSame(targets[existingIndex], { ...all })) {
        targets[existingIndex] = { ...all };
      }
    } else {
      targets.push({ ...all });
    }
    if (!areObjectsSame(targets, this.state.globalTargets)) {
      this.setState({ globalTargets: targets });
    }
  };

  updatePaths = (path: Aircraft.PathPayload) => {
    const { ...all } = path;
    let paths = [...this.state.paths];
    const existingIndex = paths.findIndex((p) => path.agentID === p.agentID);
    if (existingIndex !== -1) {
      if (!areObjectsSame(paths[existingIndex], { ...all })) {
        paths[existingIndex] = { ...all };
      }
    } else {
      paths.push({ ...all });
    }
    if (!areObjectsSame(paths, this.state.paths)) {
      this.setState({ paths });
    }
  };
  updateIcons = (icon: Environment.IconPayload) => {
    const { ...all } = icon;
    const icons = [...this.state.icons];
    const existingIndex = icons.findIndex((i) => icon.name === i.name);
    if (existingIndex !== -1) {
      if (!areObjectsSame(icons[existingIndex], { ...all })) {
        icons[existingIndex] = { ...all };
      }
    } else {
      icons.push({ ...all });
    }
    if (!areObjectsSame(icons, this.state.icons)) {
      this.setState({ icons });
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
  sendToMACE = (command: string, filteredAircrafts: Aircraft.AircraftPayload[], payload: string[]) => {
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
          removeIcon: this.removeIcon,
          setGlobalZoom: this.setZoom,
          sendToMACE: this.sendToMACE,
          updateSelectedAircraft: this.updateSelectedAircraft
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
