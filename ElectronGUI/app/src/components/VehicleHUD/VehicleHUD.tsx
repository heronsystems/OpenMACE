import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import * as MUIColors from "material-ui/styles/colors";
import Avatar from "material-ui/Avatar";
import { Card, CardActions, CardHeader, CardText } from "material-ui/Card";
import FlatButton from "material-ui/FlatButton";
import IconButton from "material-ui/IconButton";
import IconMenu from "material-ui/IconMenu";
import Menu from "material-ui/Menu";
import MenuItem from "material-ui/MenuItem";
import * as React from "react";
import { Colors } from "../../util/misc/Colors";
import { textSeverityToColor } from "../../util/misc/Colors";
import { aircraftImgSrcFromType } from "../../util/Helpers/VehicleHelper";
import { Vehicle } from "../../util/Vehicle/Vehicle";
import { styles } from "./styles";

type Props = {
    vehicleID: string;
    aircraft: Vehicle;
    highlightColor: string;
    handleAircraftCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void;
    handleChangeSelectedVehicle: (vehicleID: string) => void;
};

type State = {
    selectedBattery?: string;
    batteryText?: string;
    showHUDMessage?: boolean;
    vehicleModeText?: string;
    openVehicleMode?: boolean;
    heartbeatSyncRequested?: boolean;
};

export class VehicleHUD extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
            selectedBattery: "Voltage",
            batteryText: this.props.aircraft.fuel.batteryVoltage + " V",
            showHUDMessage: false,
            vehicleModeText: this.props.aircraft.vehicleMode,
            openVehicleMode: false,
            heartbeatSyncRequested: false
        };
    }

    componentWillReceiveProps(nextProps: Props) {
        this.handleBatteryChange(this.state.selectedBattery);

        let now = new Date();
        if (nextProps.aircraft.messages.length > 0) {
            let diff = Math.abs(now.getTime() - nextProps.aircraft.messages[0].timestamp.getTime());
            if (diff > 10000) {
                this.setState({ showHUDMessage: false });
            } else {
                this.setState({ showHUDMessage: true });
            }
        }

        if (nextProps.aircraft.vehicleMode !== this.props.aircraft.vehicleMode) {
            this.setState({ vehicleModeText: nextProps.aircraft.vehicleMode });
        }

        const lastHeardSeconds = (now.getTime() - this.props.aircraft.general.lastHeard.getTime()) / 1000; // Time in seconds
        if (lastHeardSeconds <= 10) {
            if (this.state.heartbeatSyncRequested === true) {
                this.setState({ heartbeatSyncRequested: false });
            }
        } else if (lastHeardSeconds > 30) {
            if (this.state.heartbeatSyncRequested === false) {
                this.syncVehicle();
                this.setState({ heartbeatSyncRequested: true });
            }
        }
    }

    handleLoiter = () => {
        this.props.handleAircraftCommand(this.props.vehicleID, "ISSUE_COMMAND", "AUTO_PAUSE");
    };
    handleRTL = () => {
        this.props.handleAircraftCommand(this.props.vehicleID, "ISSUE_COMMAND", "RTL");
    };
    syncVehicle = () => {
        this.props.handleAircraftCommand(this.props.vehicleID, "ISSUE_COMMAND", "FORCE_DATA_SYNC");

        // this.props.handleAircraftCommand(this.props.vehicleID, "GET_VEHICLE_HOME", "");
        // this.props.handleAircraftCommand(this.props.vehicleID, "GET_VEHICLE_MISSION", "");
    };

    handleBatteryChange = (value: string) => {
        let battText = "";
        if (value === "Voltage") {
            battText = this.props.aircraft.fuel.batteryVoltage + " V";
        } else if (value === "Percent") {
            battText = this.props.aircraft.fuel.batteryRemaining + "%";
        } else if (value === "Current") {
            battText = this.props.aircraft.fuel.batteryCurrent + " A";
        }

        this.setState({
            selectedBattery: value,
            batteryText: battText
        });
    };

    handleModeChange = (value: any) => {
        // let modeText = this.props.aircraft.vehicleMode;
        console.log("Handle mode change: " + value);
        this.props.handleAircraftCommand(this.props.vehicleID, "SET_VEHICLE_MODE", value);
    };

    render() {
        // TODO: Figure out how to move this style to a separate styles file. The conditional is hard:
        const boxShadow = this.props.aircraft.isSelected
            ? this.props.highlightColor + " 0px 1px 20px, rgba(0, 0, 0, .5) 0px 1px 4px"
            : "rgba(0, 0, 0, 0.117647) 0px 1px 6px, rgba(0, 0, 0, 0.117647) 0px 1px 4px";
        const hudStyle = {
            position: "relative" as "relative",
            width: 90 + "%",
            marginBottom: 15,
            boxShadow: boxShadow
        };

        // const hudAvatar = aircraftImgSrcFromType(this.props.aircraft.general.aircraftType);
        const hudAvatar: JSX.Element = (
            <Avatar
                backgroundColor={this.props.aircraft.isArmed ? Colors.Success : Colors.Primary}
                src={aircraftImgSrcFromType(this.props.aircraft.general.aircraftType)}
                style={styles.avatar}
            />
        );

        let now = new Date();
        const lastHeardSeconds = (now.getTime() - this.props.aircraft.general.lastHeard.getTime()) / 1000; // Time in seconds
        let heartbeatColor = MUIColors.green500;
        if (lastHeardSeconds > 10 && lastHeardSeconds <= 30) {
            heartbeatColor = MUIColors.orange500;
        } else if (lastHeardSeconds > 30) {
            heartbeatColor = MUIColors.red500;
            if (this.state.heartbeatSyncRequested === false) {
                this.syncVehicle();
            }
        }

        let modeMenuItems = [];
        for (let index in this.props.aircraft.availableModes) {
            if (index) {
                let mode = this.props.aircraft.availableModes[index];
                modeMenuItems.push(
                    <MenuItem
                        key={mode}
                        onClick={() => this.handleModeChange(mode)}
                        style={{
                            color: "rgba(0,0,0,0.7)",
                            paddingLeft: 0,
                            paddingRight: 0,
                            paddingTop: 0,
                            paddingBottom: 0,
                            marginLeft: 0,
                            marginRight: 0
                        }}
                        value={mode}
                        primaryText={mode}
                        label={mode}
                    />
                );
            }
        }

        const hudTitle = (
            <div className="row">
                <div className="col-xs-6">
                    <div className="col-xs-12">
                        <span>ID: {this.props.vehicleID}</span>
                    </div>
                    <div className="col-xs-12" style={styles.col_flat}>
                        <Menu desktop={true} style={styles.menuStyle}>
                            <MenuItem
                                style={styles.menuItem}
                                primaryText={this.props.aircraft.vehicleMode}
                                rightIcon={
                                    <i style={styles.iconStyle} className="material-icons">
                                        arrow_drop_down
                                    </i>
                                }
                                menuItems={[modeMenuItems]}
                            />
                        </Menu>
                    </div>
                </div>
                <div className="col-xs-6" style={styles.col_thin}>
                    <div className="col-xs-12" style={styles.col_thin}>
                        {/* TODO: Figure out how to move this heartbeat color to separate stylesheet */}
                        <div
                            className="col-xs-6"
                            style={{
                                color: heartbeatColor,
                                display: "flex",
                                justifyContent: "flex-end",
                                alignItems: "center",
                                paddingLeft: 0,
                                paddingRight: 0
                            }}
                        >
                            <i className="material-icons">favorite</i>
                        </div>
                        <div
                            className="col-xs-6"
                            style={{
                                display: "flex",
                                justifyContent: "center",
                                alignItems: "center",
                                paddingLeft: 0,
                                paddingRight: 0,
                                paddingTop: 4 + "px"
                            }}
                        >
                            <span
                                style={{
                                    fontSize: 12,
                                    fontWeight: "bold",
                                    color: heartbeatColor
                                }}
                            >
                                {lastHeardSeconds + " s"}
                            </span>
                        </div>
                    </div>
                    <div className="col-xs-12" style={styles.col_thin}>
                        <div className="col-xs-6" style={styles.col_icon}>
                            <i className="material-icons">gps_fixed</i>
                        </div>
                        <div className="col-xs-6" style={styles.col_iconLabel}>
                            <span style={styles.iconLabelText}>{this.props.aircraft.gps.gpsFix}</span>
                        </div>
                    </div>
                    <div className="col-xs-12" style={styles.col_thin}>
                        <div className="col-xs-6" style={styles.col_icon}>
                            <i className="material-icons">satellite</i>
                        </div>
                        <div className="col-xs-6" style={styles.col_iconLabel}>
                            <span style={styles.iconLabelText}>{this.props.aircraft.gps.visibleSats}</span>
                        </div>
                    </div>
                </div>
            </div>
        );

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Card expanded={true} onExpandChange={() => this.props.handleChangeSelectedVehicle(this.props.vehicleID)} style={hudStyle}>
                    <CardHeader titleStyle={styles.cardHeaderText} title={hudTitle} avatar={hudAvatar} actAsExpander={true} showExpandableButton={false} />

                    <div className="row">
                        <div className="col-xs-6">
                            <div className="box">
                                {/*
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Lat: " + this.props.aircraft.position.lat.toFixed(2)}
                                </CardText>
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Lon: " + this.props.aircraft.position.lon.toFixed(2)}
                                </CardText>
                                */}
                                <CardText style={styles.cardText}>{"Alt: " + this.props.aircraft.position.alt.toFixed(2)}</CardText>
                                <CardText style={styles.cardText}>{"Airspeed: " + this.props.aircraft.airspeed.toFixed(2)}</CardText>
                                <div className="col-xs-5" style={styles.batteryContainer}>
                                    <IconMenu
                                        style={styles.batteryMenu}
                                        iconButtonElement={
                                            <IconButton style={{ padding: 0 }}>
                                                <i className="material-icons">battery_charging_full</i>
                                                <i className="material-icons">arrow_drop_down</i>
                                            </IconButton>
                                        }
                                        onChange={(e: any, value: string) => this.handleBatteryChange(value)}
                                        value={this.state.selectedBattery}
                                    >
                                        <MenuItem value="Voltage" primaryText="Voltage" label="Voltage (V)" />
                                        <MenuItem value="Percent" primaryText="Percent" label="Percent (%)" />
                                        <MenuItem value="Current" primaryText="Current" label="Current (A)" />
                                    </IconMenu>
                                </div>
                                <div className="col-xs-7" style={styles.batteryText}>
                                    {this.state.batteryText}
                                </div>
                            </div>
                        </div>
                        <div className="col-xs-6">
                            <div className="box">
                                <CardText style={styles.rpyText}>{"Roll: " + this.props.aircraft.attitude.roll.toFixed(2)}</CardText>
                                <CardText style={styles.rpyText}>{"Pitch: " + this.props.aircraft.attitude.pitch.toFixed(2)}</CardText>
                                <CardText style={styles.rpyText}>{"Yaw: " + this.props.aircraft.attitude.yaw.toFixed(2)}</CardText>
                            </div>
                        </div>

                        {this.props.aircraft.messages.length > 0 &&
                            this.state.showHUDMessage && (
                                <div className="col-xs-12">
                                    <br />
                                    <div style={styles.messagesContainer}>
                                        <span style={styles.messagesTitle}>Messages</span>
                                    </div>
                                    <br />
                                    <div className="box">
                                        <CardText
                                            style={{
                                                color: textSeverityToColor(this.props.aircraft.messages[0].severity),
                                                fontSize: 18,
                                                paddingTop: 0,
                                                paddingBottom: 0
                                            }}
                                        >
                                            {"[" + this.props.aircraft.messages[0].severity + "] : " + this.props.aircraft.messages[0].text}
                                        </CardText>
                                    </div>
                                    <br />
                                    <div style={styles.borderLine} />
                                </div>
                            )}
                    </div>

                    <CardActions style={styles.cardActions}>
                        <FlatButton label="Loiter" onClick={this.handleLoiter} icon={<i className="material-icons">pause</i>} />
                        <FlatButton label="Home" onClick={this.handleRTL} icon={<i className="material-icons">home</i>} />
                        <FlatButton label="Sync" onClick={this.syncVehicle} icon={<i className="material-icons">cached</i>} />
                    </CardActions>
                </Card>
            </MuiThemeProvider>
        );
    }
}
