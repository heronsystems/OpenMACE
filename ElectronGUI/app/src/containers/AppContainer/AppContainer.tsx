import * as React from "react";

import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import * as GlobalTypes from "../../types/globalTypings";
import { styles } from "./styles";

let NotificationSystem = require("react-notification-system");
import * as deepcopy from "deepcopy";
import AppBar from "material-ui/AppBar";
// import FlatButton from "material-ui/FlatButton";
import { ConfigDialog } from "../../components/Dialogs/ConfigDialog/ConfigDialog";
import { GlobalOriginDialog } from "../../components/Dialogs/GlobalOriginDialog/GlobalOriginDialog";
import { MessagesDialog } from "../../components/Dialogs/MessagesDialog/MessagesDialog";
import { TakeoffDialog } from "../../components/Dialogs/TakeoffDialog/TakeoffDialog";
import { VehicleHomeDialog } from "../../components/Dialogs/VehicleHomeDialog/VehicleHomeDialog";
import MACEMap from "../../components/MACEMap/MACEMap";
import { EnvironmentSettings } from "../../components/Settings/EnvironmentSettings";
import { AppHelper } from "../../util/Helpers/AppHelper";
import { MACECommsHelper } from "../../util/Helpers/MACECommsHelper";
import { PolygonHelper } from "../../util/Helpers/PolygonHelper";
import { Vehicle } from "../../util/Vehicle/Vehicle";
import { AppDrawer } from "../AppDrawer/AppDrawer";
import { ConnectedVehiclesContainer } from "../ConnectedVehicles/ConnectedVehiclesContainer";
import { DrawButtonsContainer } from "../DrawButtons/DrawButtonsContainer";
import { VehicleCommandsContainer } from "../VehicleCommands/VehicleCommandsContainer";

let injectTapEventPlugin = require("react-tap-event-plugin");
injectTapEventPlugin();

import * as L from "leaflet";

// // Performance testing:
// var Perf = require('react-addons-perf');
// // End performance testing

type Props = {};

type State = {
    selectedVehicleID?: string;
    openDrawer?: boolean;
    showEditVehicleHomeDialog?: boolean;
    showEditGlobalHomeDialog?: boolean;
    showMessagesMenu?: boolean;
    showConfigDialog?: boolean;
    showTakeoffDialog?: boolean;
    showSaveTakeoff?: boolean;
    showEnvironmentSettings?: boolean;
    getConnectedVehiclesTimeout?: number;
    useContext?: boolean;
    contextAnchor?: L.LeafletMouseEvent;
    connectedVehicles?: { [id: string]: Vehicle };
};

export default class AppContainer extends React.Component<Props, State> {
    notificationSystem: any; // TODO: Figure out why I cant make this a NotificationSystem type...
    getVehiclesInterval: any;
    logger: any;
    appHelper: AppHelper;
    maceCommsHelper: MACECommsHelper;
    polygonHelper: PolygonHelper;

    constructor(props: Props) {
        super(props);

        this.getVehiclesInterval = null;

        this.state = {
            selectedVehicleID: "0",
            openDrawer: false,
            showEditVehicleHomeDialog: false,
            showEditGlobalHomeDialog: false,
            showMessagesMenu: false,
            showConfigDialog: false,
            showTakeoffDialog: false,
            showSaveTakeoff: false,
            showEnvironmentSettings: false,
            getConnectedVehiclesTimeout: 3000,
            useContext: false,
            connectedVehicles: {}
        };

        this.appHelper = new AppHelper(this.state);
        this.polygonHelper = new PolygonHelper();
    }

    componentDidMount() {
        // // Performance testing:
        // setTimeout(() => {
        //   console.log("Start performance testing...");
        //   Perf.start();
        //   setTimeout(() => {
        //     Perf.stop();
        //     const measurements = Perf.getLastMeasurements();
        //     Perf.printWasted(measurements);
        //   }, 30000);
        // }, 5000);
        // // End performance testing

        this.notificationSystem = this.refs.notificationSystem;

        this.appHelper.maceCommsHelper.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");

        this.getVehiclesInterval = setInterval(() => {
            this.appHelper.maceCommsHelper.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
        }, this.appHelper.getConnectedVehiclesTimeout);

        // Set interval to set state to DB:
        setInterval(() => {
            if (!this.appHelper.pauseMACEComms) {
                this.setState({
                    connectedVehicles: this.appHelper.maceCommsHelper.vehicleDB.vehicles
                });
            }
        }, 1500);
    }

    showNotification = (title: string, message: string, level: string, position: string, label: string) => {
        let notification = {
            title: title,
            message: message,
            level: level,
            position: position,
            action: {
                label: label
            }
        };

        this.notificationSystem.addNotification(notification);
    };

    handleAircraftCommand = (id: string, tcpCommand: string, vehicleCommand: string) => {
        console.log(tcpCommand);
        this.appHelper.maceCommsHelper.makeTCPRequest(parseInt(id), tcpCommand, vehicleCommand);
    };

    handleDrawerAction = (action: string) => {
        if (action === "MACEConfig") {
            this.setState({
                showMessagesMenu: false,
                showConfigDialog: true,
                showTakeoffDialog: false,
                showSaveTakeoff: false,
                openDrawer: false
            });
            this.appHelper.pauseMACEComms = true;
        } else if (action === "Messages") {
            this.setState({
                showMessagesMenu: true,
                showConfigDialog: false,
                showTakeoffDialog: false,
                showSaveTakeoff: false,
                openDrawer: false
            });
            this.appHelper.pauseMACEComms = false;
        } else if (action === "Takeoff") {
            this.setState({
                showMessagesMenu: false,
                showConfigDialog: false,
                showTakeoffDialog: true,
                showSaveTakeoff: true,
                openDrawer: false
            });
            this.appHelper.pauseMACEComms = false;
        } else if (action === "TestButton1") {
            this.appHelper.maceCommsHelper.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION1", "");
        } else if (action === "TestButton2") {
            this.appHelper.maceCommsHelper.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION2", "");
        } else if (action === "EditEnvironment") {
            // Ask for global origin:
            this.appHelper.maceCommsHelper.makeTCPRequest(0, "GET_GLOBAL_ORIGIN", "");

            this.setState({ openDrawer: false });
            this.polygonHelper.showDraw = true;
        }
    };

    contextSetHome = () => {
        this.setState({
            showEditVehicleHomeDialog: true,
            showEditGlobalHomeDialog: false,
            showTakeoffDialog: false,
            useContext: true
        });
        this.appHelper.allowVehicleSelect = true;
        this.appHelper.pauseMACEComms = true;
    };

    contextSetGlobal = () => {
        this.setState({
            showEditGlobalHomeDialog: true,
            showEditVehicleHomeDialog: false,
            useContext: true
        });
        this.appHelper.allowVehicleSelect = false;
        this.appHelper.pauseMACEComms = true;
    };

    contextSetTakeoff = () => {
        this.setState({
            showEditVehicleHomeDialog: false,
            showEditGlobalHomeDialog: false,
            showTakeoffDialog: true,
            useContext: true
        });
        this.appHelper.allowVehicleSelect = false;
        this.appHelper.pauseMACEComms = true;
    };

    contextGoHere = () => {
        this.setState({
            showEditGlobalHomeDialog: false,
            showEditVehicleHomeDialog: false,
            useContext: true
        });
        this.appHelper.allowVehicleSelect = false;
        let goHere = {
            lat: this.state.contextAnchor.latlng.lat,
            lon: this.state.contextAnchor.latlng.lng
        };
        this.handleAircraftCommand(this.state.selectedVehicleID, "SET_GO_HERE", JSON.stringify(goHere));
    };

    handleSelectedAircraftUpdate = (id: string) => {
        let stateCopy = deepcopy(this.state.connectedVehicles);
        let selectedID = "0";
        Object.keys(this.state.connectedVehicles).map((key: string) => {
            if (key === id) {
                stateCopy[id].isSelected = !stateCopy[id].isSelected;
                selectedID = stateCopy[id].isSelected ? id : "0";

                if (stateCopy[id].isSelected === true && (stateCopy[id].position.lat !== 0 && stateCopy[id].position.lat !== 0)) {
                    // this.setState({mapCenter: [stateCopy[id].position.lat, stateCopy[id].position.lon]});
                }
            } else {
                stateCopy[key].isSelected = false;
            }
            stateCopy[key].updateHomePosition();
            stateCopy[key].updateVehicleMarkerPosition();
        });

        this.appHelper.maceCommsHelper.vehicleDB.vehicles = stateCopy;
        this.setState({ selectedVehicleID: selectedID });
    };

    updateMapCenter = (e: L.DragEndEvent) => {
        this.appHelper.updateMapCenter(e);
    };

    handleSaveTakeoff = (takeoffAlt: string) => {
        this.appHelper.handleSaveTakeoff(takeoffAlt);
    };

    handleTakeoff = () => {
        this.setState({
            showTakeoffDialog: true,
            showSaveTakeoff: false,
            useContext: false
        });
        this.appHelper.pauseMACEComms = true;
    };

    handleCloseDialogs = () => {
        this.setState({
            showEditVehicleHomeDialog: false,
            useContext: false,
            showEditGlobalHomeDialog: false,
            showMessagesMenu: false,
            showConfigDialog: false,
            showTakeoffDialog: false,
            showEnvironmentSettings: false
        });
        this.appHelper.pauseMACEComms = false;
    };

    handleOpenEnvironmentSettings = () => {
        this.setState({ showEnvironmentSettings: true });
        this.appHelper.pauseMACEComms = true;
    };

    handleSubmitBoundary = () => {
        this.appHelper.maceCommsHelper.makeTCPRequest(0, "SET_ENVIRONMENT_VERTICES", JSON.stringify({ boundary: this.polygonHelper.drawPolygonPts }));
        this.polygonHelper.handleSubmitBoundary();
    };

    handleDisconnectedVehicle = (vehicleID: string) => {
        // Delete vehicle from DB:
        delete this.appHelper.maceCommsHelper.vehicleDB.vehicles[vehicleID];
    };

    render() {
        // const ToolbarRight = () => (
        //     <FlatButton
        //         label={"Sync all"}
        //         labelPosition={"before"}
        //         onClick={this.appHelper.handleSyncAll}
        //         icon={<i className="material-icons">cached</i>}
        //         style={styles.whiteButton}
        //     />
        // );

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div style={styles.parentStyle}>
                    <AppBar
                        title="MACE"
                        style={styles.appBar}
                        onLeftIconButtonClick={() =>
                            this.setState({
                                openDrawer: !this.state.openDrawer
                            })
                        }
                        // iconElementRight={<ToolbarRight />}
                    />

                    <AppDrawer
                        openDrawer={this.state.openDrawer}
                        onToggleDrawer={(open: boolean) => this.setState({ openDrawer: open })}
                        onDrawerAction={(action: string) => this.handleDrawerAction(action)}
                        showMessagesMenu={this.state.showMessagesMenu}
                    />

                    <ConnectedVehiclesContainer
                        connectedVehicles={this.state.connectedVehicles}
                        onAircraftCommand={this.handleAircraftCommand}
                        handleChangeSelectedVehicle={this.handleSelectedAircraftUpdate}
                        selectedVehicleID={this.state.selectedVehicleID}
                        onDisconnectedVehicle={this.handleDisconnectedVehicle}
                    />

                    <VehicleCommandsContainer
                        connectedVehicles={this.state.connectedVehicles}
                        onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
                        onAircraftCommand={this.handleAircraftCommand}
                        selectedAircraftID={this.state.selectedVehicleID}
                        handleTakeoff={this.handleTakeoff}
                    />

                    {this.state.showEditVehicleHomeDialog && (
                        <VehicleHomeDialog
                            open={this.state.showEditVehicleHomeDialog}
                            handleClose={this.handleCloseDialogs}
                            vehicles={this.state.connectedVehicles}
                            selectedVehicleID={this.state.selectedVehicleID}
                            handleSave={this.appHelper.handleSaveVehicleHome}
                            contextAnchor={this.state.contextAnchor}
                            useContext={this.state.useContext}
                            allowVehicleSelect={this.appHelper.allowVehicleSelect}
                            onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
                            showNotification={this.showNotification}
                        />
                    )}

                    {this.state.showEditGlobalHomeDialog && (
                        <GlobalOriginDialog
                            open={this.state.showEditGlobalHomeDialog}
                            handleClose={this.handleCloseDialogs}
                            onGlobalHomeCommand={this.handleAircraftCommand}
                            globalOrigin={this.appHelper.maceCommsHelper.vehicleDB.globalOrigin}
                            handleSave={this.appHelper.handleSaveGlobalOrigin}
                            contextAnchor={this.state.contextAnchor}
                            useContext={this.state.useContext}
                        />
                    )}

                    {this.state.showMessagesMenu && (
                        <MessagesDialog
                            open={this.state.showMessagesMenu}
                            handleClose={this.handleCloseDialogs}
                            handleSave={this.appHelper.handleSaveMessagingPreferences}
                            preferences={this.appHelper.maceCommsHelper.vehicleDB.messagePreferences}
                        />
                    )}

                    {this.state.showConfigDialog && (
                        <ConfigDialog
                            open={this.state.showConfigDialog}
                            handleClose={this.handleCloseDialogs}
                            handleSave={(configSettings: GlobalTypes.ConfigSettingsType, reload: boolean) =>
                                this.appHelper.handleSaveMACEConfig(configSettings, reload)
                            }
                            configSettings={this.appHelper.MACEconfig}
                            handleParseJSON={(filename: string, restartServer: boolean) => this.appHelper.parseJSONConfig(filename, restartServer)}
                        />
                    )}

                    {this.state.showTakeoffDialog && (
                        <TakeoffDialog
                            open={this.state.showTakeoffDialog}
                            handleClose={this.handleCloseDialogs}
                            vehicles={this.state.connectedVehicles}
                            selectedVehicleID={this.state.selectedVehicleID}
                            handleTakeoff={this.appHelper.handleTakeoff}
                            takeoffAlt={this.appHelper.MACEconfig.config.VehicleSettings.defaultTakeoffAlt.toString()}
                            onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
                            showSaveTakeoff={this.state.showSaveTakeoff}
                            handleSaveTakeoff={(alt: string) => this.handleSaveTakeoff(alt)}
                            contextAnchor={this.state.contextAnchor}
                            useContext={this.state.useContext}
                            showNotification={this.showNotification}
                        />
                    )}

                    {this.polygonHelper.showDraw && (
                        <DrawButtonsContainer
                            onDeleteLastPolygonPt={this.polygonHelper.handleDeleteLastPolygonPt}
                            onDisableDraw={this.polygonHelper.handleDisableDraw}
                            onSubmitBoundary={this.handleSubmitBoundary}
                            onClearAllPts={this.polygonHelper.handleClearPts}
                            handleChangeGridSpacing={this.polygonHelper.handleChangeGridSpacing}
                            openEnvironmentSettings={this.handleOpenEnvironmentSettings}
                            environmentSettings={this.polygonHelper.environmentSettings}
                        />
                    )}

                    {this.state.showEnvironmentSettings && (
                        <EnvironmentSettings
                            open={this.state.showEnvironmentSettings}
                            handleClose={this.handleCloseDialogs}
                            handleSave={this.polygonHelper.saveEnvironmentSettings}
                            environmentSettings={this.polygonHelper.environmentSettings}
                        />
                    )}

                    <MACEMap
                        handleSelectedAircraftUpdate={this.handleSelectedAircraftUpdate}
                        setContextAnchor={(anchor: L.LeafletMouseEvent) => this.setState({ contextAnchor: anchor })}
                        connectedVehicles={this.state.connectedVehicles}
                        selectedVehicleID={this.state.selectedVehicleID}
                        mapCenter={this.appHelper.MACEconfig.config.GUIInit.mapCenter}
                        maxZoom={this.appHelper.MACEconfig.config.GUIInit.maxZoom}
                        mapZoom={this.appHelper.MACEconfig.config.GUIInit.mapZoom}
                        globalOrigin={this.appHelper.maceCommsHelper.vehicleDB.globalOrigin}
                        updateMapCenter={this.updateMapCenter}
                        contextAnchor={this.state.contextAnchor}
                        contextSetGlobal={this.contextSetGlobal}
                        contextSetHome={this.contextSetHome}
                        contextSetTakeoff={this.contextSetTakeoff}
                        contextGoHere={this.contextGoHere}
                        environmentBoundary={this.appHelper.maceCommsHelper.vehicleDB.environmentBoundary}
                        drawPolygonPts={this.polygonHelper.drawPolygonPts}
                        onAddPolygonPt={this.polygonHelper.handleAddPolygonPt}
                        environmentSettings={this.polygonHelper.environmentSettings}
                        gridPts={this.polygonHelper.gridPts}
                        envBoundingBox={this.polygonHelper.envBoundingBox}
                    />

                    <div>
                        <NotificationSystem ref="notificationSystem" />
                    </div>
                </div>
            </MuiThemeProvider>
        );
    }
}
