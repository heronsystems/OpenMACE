let fs = electronRequire("fs");
import * as L from "leaflet";
import * as GlobalTypes from "../../types/globalTypings";
import { Vehicle } from "../Vehicle/Vehicle";
import { MACECommsHelper } from "./MACECommsHelper";

/*
  TODO:
    1) Figure out how to move notifications out of AppContainer.tsx?
*/

export class AppHelper {
    state: any; // TODO: Figure out how to get State type in here...
    MACEconfig: GlobalTypes.ConfigSettingsType;
    notificationSystem: any; // TODO: Figure out why I cant make this a NotificationSystem type...
    getVehiclesInterval: any;
    connectedVehicles: { [id: string]: Vehicle };
    selectedVehicleID?: string;
    openDrawer?: boolean;
    allowVehicleSelect?: boolean;
    showEditVehicleHomeDialog?: boolean;
    showEditGlobalHomeDialog?: boolean;
    showMessagesMenu?: boolean;
    showConfigDialog?: boolean;
    showTakeoffDialog?: boolean;
    showSaveTakeoff?: boolean;
    useContext?: boolean;
    contextAnchor?: L.LeafletMouseEvent;
    pauseMACEComms?: boolean;
    getConnectedVehiclesTimeout?: number;

    maceCommsHelper?: MACECommsHelper;

    constructor(appState: any) {
        this.state = appState;
        this.getVehiclesInterval = null;
        this.MACEconfig = {
            filename: "../GUIConfig.json",
            config: {
                MACEComms: {
                    ipAddress: "127.0.0.1",
                    listenPortNumber: 1234,
                    sendPortNumber: 5678
                },
                GUIInit: {
                    mapCenter: { lat: 37.889231, lng: -76.810302, alt: 0 }, // Bob's Farm
                    // mapCenter: [-35.363272, 149.165249], // SITL Default
                    // mapCenter: [45.283410, -111.400850], // Big Sky
                    mapZoom: 20,
                    maxZoom: 21
                },
                VehicleSettings: {
                    defaultTakeoffAlt: 5
                }
            }
        };
        this.openDrawer = false;
        this.allowVehicleSelect = true;
        this.showEditVehicleHomeDialog = false;
        this.showEditGlobalHomeDialog = false;
        this.selectedVehicleID = "0";
        this.showMessagesMenu = false;
        this.showConfigDialog = false;
        this.showTakeoffDialog = false;
        this.showSaveTakeoff = false;
        this.pauseMACEComms = false;
        this.getConnectedVehiclesTimeout = 3000;

        // Initialize MACE Comms helper:
        this.maceCommsHelper = new MACECommsHelper(this.MACEconfig);

        // Parse JSON File:
        this.parseJSONConfig(this.MACEconfig.filename);

        // this.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
        this.maceCommsHelper.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");

        this.getVehiclesInterval = setInterval(() => {
            this.maceCommsHelper.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
        }, this.state.getConnectedVehiclesTimeout);
    }

    parseJSONConfig = (filename: string, restartServer: boolean = true) => {
        let jsonConfig = JSON.parse(fs.readFileSync(filename));
        let MACEconfig: GlobalTypes.ConfigSettingsType = this.MACEconfig;

        MACEconfig.filename = filename;
        if (jsonConfig.MACEComms) {
            if (jsonConfig.MACEComms.ipAddress) {
                MACEconfig.config.MACEComms.ipAddress = jsonConfig.MACEComms.ipAddress;
                if (restartServer) {
                    this.maceCommsHelper.setupTCPServer();
                }

                // Reset interval and start requests again:
                if (this.getVehiclesInterval) {
                    clearInterval(this.getVehiclesInterval);
                    this.getVehiclesInterval = setInterval(() => {
                        this.maceCommsHelper.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
                    }, this.state.getConnectedVehiclesTimeout);
                }
            }
            if (jsonConfig.MACEComms.listenPortNumber) {
                MACEconfig.config.MACEComms.listenPortNumber = jsonConfig.MACEComms.listenPortNumber;
                if (restartServer) {
                    this.maceCommsHelper.setupTCPServer();
                }
            }
            if (jsonConfig.MACEComms.sendPortNumber) {
                MACEconfig.config.MACEComms.sendPortNumber = jsonConfig.MACEComms.sendPortNumber;

                // Reset interval and start requests again:
                if (this.getVehiclesInterval) {
                    clearInterval(this.getVehiclesInterval);
                    this.getVehiclesInterval = setInterval(() => {
                        this.maceCommsHelper.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
                    }, this.state.getConnectedVehiclesTimeout);
                }
            }
        }
        if (jsonConfig.GUIInit) {
            if (jsonConfig.GUIInit.mapCenter) {
                let center = { lat: jsonConfig.GUIInit.mapCenter.lat, lng: jsonConfig.GUIInit.mapCenter.lng, alt: 0 };
                MACEconfig.config.GUIInit.mapCenter = center;
            }
            if (jsonConfig.GUIInit.mapZoom) {
                MACEconfig.config.GUIInit.mapZoom = jsonConfig.GUIInit.mapZoom;
            }
            if (jsonConfig.GUIInit.maxZoom) {
                MACEconfig.config.GUIInit.maxZoom = jsonConfig.GUIInit.maxZoom;
            }
        }
        if (jsonConfig.VehicleSettings) {
            if (jsonConfig.VehicleSettings.defaultTakeoffAlt) {
                MACEconfig.config.VehicleSettings.defaultTakeoffAlt = jsonConfig.VehicleSettings.defaultTakeoffAlt;
            }
        }

        this.MACEconfig = MACEconfig;
    };

    handleClearGUI = () => {
        this.maceCommsHelper.vehicleDB.vehicles = {};
        this.state.selectedVehicleID = "0";
    };

    handleAircraftCommand = (id: string, tcpCommand: string, vehicleCommand: string) => {
        console.log(tcpCommand);
        this.maceCommsHelper.makeTCPRequest(parseInt(id), tcpCommand, vehicleCommand);
    };

    handleDrawerAction = (action: string) => {
        if (action === "MACEConfig") {
            this.state.showMessagesMenu = false;
            this.state.showConfigDialog = true;
            this.state.showTakeoffDialog = false;
            this.state.showSaveTakeoff = false;
            this.state.openDrawer = false;
            this.pauseMACEComms = true;
        } else if (action === "Messages") {
            this.state.showMessagesMenu = true;
            this.state.showConfigDialog = false;
            this.state.showTakeoffDialog = false;
            this.state.showSaveTakeoff = false;
            this.state.openDrawer = false;
            this.pauseMACEComms = false;
        } else if (action === "Takeoff") {
            this.state.showMessagesMenu = false;
            this.state.showConfigDialog = false;
            this.state.showTakeoffDialog = true;
            this.state.showSaveTakeoff = true;
            this.state.openDrawer = false;
            this.pauseMACEComms = true;
        } else if (action === "TestButton1") {
            this.maceCommsHelper.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION1", "");
        } else if (action === "TestButton2") {
            this.maceCommsHelper.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION2", "");
        } else if (action === "EditEnvironment") {
            // Ask for global origin:
            this.maceCommsHelper.makeTCPRequest(0, "GET_GLOBAL_ORIGIN", "");

            this.state.showDraw = true;
            this.state.openDrawer = false;
        }
    };

    handleSaveVehicleHome = (vehicleID: string, vehicleHome: GlobalTypes.PositionType) => {
        this.handleAircraftCommand(vehicleID, "SET_VEHICLE_HOME", JSON.stringify(vehicleHome));
    };

    handleSaveGlobalOrigin = (globalOrigin: GlobalTypes.PositionType) => {
        this.handleAircraftCommand("0", "SET_GLOBAL_ORIGIN", JSON.stringify(globalOrigin));
        // this.maceCommsHelper.vehicleDB.globalOrigin = globalOrigin;
    };

    contextSetHome = () => {
        this.state.showEditVehicleHomeDialog = true;
        this.state.allowVehicleSelect = true;
        this.state.showEditGlobalHomeDialog = false;
        this.state.showTakeoffDialog = false;
        this.state.useContext = true;
        this.pauseMACEComms = true;
    };

    contextSetGlobal = () => {
        this.state.showEditGlobalHomeDialog = true;
        this.state.allowVehicleSelect = false;
        this.state.showEditVehicleHomeDialog = false;
        this.state.useContext = true;
        this.pauseMACEComms = true;
    };

    contextSetTakeoff = () => {
        this.state.showEditVehicleHomeDialog = false;
        this.state.allowVehicleSelect = false;
        this.state.showEditGlobalHomeDialog = false;
        this.state.showTakeoffDialog = true;
        this.state.useContext = true;
        this.pauseMACEComms = true;
    };

    contextGoHere = () => {
        this.state.showEditGlobalHomeDialog = false;
        this.state.allowVehicleSelect = false;
        this.state.showEditVehicleHomeDialog = false;
        this.state.useContext = true;
        let goHere = {
            lat: this.state.contextAnchor.latlng.lat,
            lon: this.state.contextAnchor.latlng.lng
        };
        this.handleAircraftCommand(this.state.selectedVehicleID, "SET_GO_HERE", JSON.stringify(goHere));
    };

    handleSaveMessagingPreferences = (preferences: GlobalTypes.MessagePreferencesType) => {
        this.maceCommsHelper.vehicleDB.messagePreferences = preferences;
    };

    handleSaveMACEConfig = (config: GlobalTypes.ConfigSettingsType, reload: boolean = false) => {
        this.MACEconfig = config;
        if (reload) {
            this.maceCommsHelper.setupTCPServer();
        }
        // Reset interval and start requests again:
        if (this.getVehiclesInterval) {
            clearInterval(this.getVehiclesInterval);
            this.getVehiclesInterval = setInterval(() => {
                this.maceCommsHelper.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
            }, this.state.getConnectedVehiclesTimeout);
        }
    };

    handleTakeoff = (vehicleID: string, takeoffAlt: string, takeoffLat?: string, takeoffLon?: string) => {
        let takeoffPosition = {
            lat: takeoffLat ? parseFloat(takeoffLat) : 0,
            lon: takeoffLon ? parseFloat(takeoffLon) : 0,
            alt: parseFloat(takeoffAlt)
        };
        let latLonFlag = false;
        if (takeoffLat && takeoffLon) {
            latLonFlag = true;
        }
        this.maceCommsHelper.makeTCPRequest(
            parseInt(vehicleID),
            "VEHICLE_TAKEOFF",
            JSON.stringify({ takeoffPosition: takeoffPosition, latLonFlag: latLonFlag })
        );
    };

    updateMapCenter = (e: L.DragEndEvent) => {
        let MACEconfig = this.MACEconfig;
        MACEconfig.config.GUIInit.mapCenter = { lat: e.target.getCenter().lat, lng: e.target.getCenter().lng, alt: 0 };
        MACEconfig.config.GUIInit.mapZoom = e.target.getZoom();
        this.MACEconfig = MACEconfig;
    };

    handleSyncAll = () => {
        this.maceCommsHelper.makeTCPRequest(0, "GET_ENVIRONMENT_BOUNDARY", "");

        this.maceCommsHelper.makeTCPRequest(0, "ISSUE_COMMAND", "FORCE_DATA_SYNC");
    };

    handleSaveTakeoff = (takeoffAlt: string) => {
        let MACEconfig = this.MACEconfig;
        MACEconfig.config.VehicleSettings.defaultTakeoffAlt = parseFloat(takeoffAlt);
        this.MACEconfig = MACEconfig;
    };
}
