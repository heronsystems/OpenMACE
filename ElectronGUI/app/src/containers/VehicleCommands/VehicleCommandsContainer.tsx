import getMuiTheme from "material-ui/styles/getMuiTheme"
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider"
const lightMuiTheme = getMuiTheme()
import * as React from "react"

import DropDownMenu from "material-ui/DropDownMenu"
import MenuItem from "material-ui/MenuItem"
import RaisedButton from "material-ui/RaisedButton"
// import * as colors from 'material-ui/styles/colors';
// import FontIcon from 'material-ui/FontIcon';
// import { Grid, Col, Row } from 'react-bootstrap';

import { Vehicle } from "../../util/Vehicle/Vehicle"
import { styles } from "./styles"

type Props = {
    connectedVehicles: { [id: string]: Vehicle }
    selectedAircraftID: string
    onSelectedAircraftChange: (id: string) => void
    onAircraftCommand: (
        id: string,
        tcpCommand: string,
        vehicleCommand: string
    ) => void
    handleTakeoff: () => void
}

type State = {
    selectedAircraftID?: string
    vehicleArmed?: boolean
    missionButtonText?: string
    missionButtonIcon?: string
}

export class VehicleCommandsContainer extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props)

        this.state = {
            selectedAircraftID: this.props.selectedAircraftID,
            vehicleArmed: false,
            missionButtonText: this.getMissionButtonText(),
            missionButtonIcon: "send"
        }
    }

    shouldComponentUpdate(nextProps: Props, state: State) {
        if (this.props.selectedAircraftID !== nextProps.selectedAircraftID) {
            return true
        }

        if (
            Object.keys(this.props.connectedVehicles).length > 0 &&
            this.props.selectedAircraftID !== "0"
        ) {
            if (
                this.props.connectedVehicles[this.props.selectedAircraftID]
                    .vehicleMission.missionState ===
                nextProps.connectedVehicles[this.props.selectedAircraftID]
                    .vehicleMission.missionState
            ) {
                return true
            }
        }

        if (
            Object.keys(this.props.connectedVehicles).length !==
            Object.keys(nextProps.connectedVehicles).length
        ) {
            return true
        }

        return false
    }

    componentWillReceiveProps(nextProps: Props) {
        if (this.props.selectedAircraftID !== nextProps.selectedAircraftID) {
            this.setState({ selectedAircraftID: nextProps.selectedAircraftID })
        }

        // if(Object.keys(nextProps.connectedVehicles).length > 0 && this.props.selectedAircraftID !== "0") {
        //     if(this.props.connectedVehicles[this.props.selectedAircraftID].vehicleMission.missionState === nextProps.connectedVehicles[this.props.selectedAircraftID].vehicleMission.missionState) {
        //         this.state.missionButtonText = this.getMissionButtonText();
        //         this.state.missionButtonIcon = this.getExecutionStateIconString();
        //     }
        // }
    }

    handleDropdownChange = (event: any, index: number, value: string) => {
        this.setState({ selectedAircraftID: value })
        this.props.onSelectedAircraftChange(value)
    }

    getExecutionStateIconString(): string {
        let iconStr = "send"
        if (Object.keys(this.props.connectedVehicles).length > 0) {
            if (
                this.props.connectedVehicles[this.props.selectedAircraftID]
                    .vehicleMission
            ) {
                if (
                    this.props.connectedVehicles[this.props.selectedAircraftID]
                        .vehicleMission.missionState === "EXECUTING"
                ) {
                    iconStr = "pause"
                } else if (
                    this.props.connectedVehicles[this.props.selectedAircraftID]
                        .vehicleMission.missionState === "PAUSED"
                ) {
                    iconStr = "play"
                }
            }
        }

        return iconStr
    }

    getMissionButtonText(): any {
        let buttonText = "Start Mission"
        if (Object.keys(this.props.connectedVehicles).length > 0) {
            if (
                this.props.connectedVehicles[this.props.selectedAircraftID]
                    .vehicleMission
            ) {
                if (
                    this.props.connectedVehicles[this.props.selectedAircraftID]
                        .vehicleMission.missionState === "EXECUTING"
                ) {
                    buttonText = "Pause Mission"
                } else if (
                    this.props.connectedVehicles[this.props.selectedAircraftID]
                        .vehicleMission.missionState === "PAUSED"
                ) {
                    buttonText = "Resume Mission"
                }
            }
        }

        return buttonText
    }

    handleStartMission = () => {
        let command = "AUTO_START"
        if (this.state.selectedAircraftID !== "0") {
            if (
                this.props.connectedVehicles[this.props.selectedAircraftID]
                    .vehicleMission.missionState === "EXECUTING"
            ) {
                command = "AUTO_PAUSE"
            } else if (
                this.props.connectedVehicles[this.props.selectedAircraftID]
                    .vehicleMission.missionState === "PAUSED"
            ) {
                command = "AUTO_RESUME"
            }
        }

        this.props.onAircraftCommand(
            this.state.selectedAircraftID.toString(),
            "ISSUE_COMMAND",
            command
        )
    }

    umdTestLand = () => {
        if (this.state.selectedAircraftID === "0") {
            for (let key in this.props.connectedVehicles) {
                if (key) {
                    this.props.onAircraftCommand(
                        key,
                        "SET_VEHICLE_MODE",
                        "LAND"
                    );
                }
            }
        }
        else {
            this.props.onAircraftCommand(
                this.state.selectedAircraftID.toString(),
                "SET_VEHICLE_MODE",
                "LAND"
            );
        }

        // this.props.onAircraftCommand(
        //     this.state.selectedAircraftID.toString(),
        //     "ISSUE_COMMAND",
        //     "LAND"
        // )
    }

    render() {
        let vehicleIDs: JSX.Element[] = []
        for (let key in this.props.connectedVehicles) {
            if (key) {
                // let vehicle = this.props.connectedVehicles[key];

                vehicleIDs.push(
                    <MenuItem
                        key={key}
                        value={key}
                        primaryText={key}
                        label={key}
                    />
                )
            }
        }

        return (
            <div>
                {Object.keys(this.props.connectedVehicles).length >= 0 && (
                    <div style={styles.aircraftCommsContainer}>
                        {Object.keys(this.props.connectedVehicles).length >
                            0 && (
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <DropDownMenu
                                    animated={false}
                                    anchorOrigin={{
                                        vertical: "bottom",
                                        horizontal: "left"
                                    }}
                                    style={styles.dropdownMenu}
                                    value={this.state.selectedAircraftID}
                                    onChange={this.handleDropdownChange}
                                >
                                    <MenuItem
                                        value={"0"}
                                        primaryText={"All vehicles"}
                                        label={"All vehicles"}
                                    />
                                    {vehicleIDs}
                                </DropDownMenu>
                            </MuiThemeProvider>
                        )}

                        {this.props.connectedVehicles[
                            this.props.selectedAircraftID
                        ] && (
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                {this.props.connectedVehicles[
                                    this.props.selectedAircraftID
                                ].isArmed ? (
                                    <RaisedButton
                                        icon={
                                            <i className="material-icons">
                                                clear
                                            </i>
                                        }
                                        style={styles.buttonStyle}
                                        label="Disarm"
                                        onClick={() =>
                                            this.props.onAircraftCommand(
                                                this.state.selectedAircraftID.toString(),
                                                "SET_VEHICLE_ARM",
                                                JSON.stringify({ arm: false })
                                            )
                                        }
                                    />
                                ) : (
                                    <RaisedButton
                                        icon={
                                            <i className="material-icons">
                                                check
                                            </i>
                                        }
                                        style={styles.buttonStyle}
                                        label="Arm"
                                        onClick={() =>
                                            this.props.onAircraftCommand(
                                                this.state.selectedAircraftID.toString(),
                                                "SET_VEHICLE_ARM",
                                                JSON.stringify({ arm: true })
                                            )
                                        }
                                    />
                                )}
                            </MuiThemeProvider>
                        )}

                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton
                                icon={
                                    <i className="material-icons">
                                        flight_takeoff
                                    </i>
                                }
                                style={styles.buttonStyle}
                                label="Takeoff"
                                onClick={this.props.handleTakeoff}
                            />
                        </MuiThemeProvider>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton
                                icon={<i className="material-icons">pause</i>}
                                style={styles.buttonStyle}
                                label="Pause"
                                onClick={() =>
                                    this.props.onAircraftCommand(
                                        this.state.selectedAircraftID.toString(),
                                        "ISSUE_COMMAND",
                                        "AUTO_PAUSE"
                                    )
                                }
                            />
                        </MuiThemeProvider>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton
                                icon={<i className="material-icons">get_app</i>}
                                style={styles.buttonStyle}
                                label="Land"
                                onClick={() =>
                                    // this.props.onAircraftCommand(
                                    //     this.state.selectedAircraftID.toString(),
                                    //     "ISSUE_COMMAND",
                                    //     "LAND"
                                    // )
                                    this.umdTestLand()

                                }
                            />
                        </MuiThemeProvider>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton
                                icon={<i className="material-icons">home</i>}
                                style={styles.buttonStyle}
                                label="Home"
                                onClick={() =>
                                    this.props.onAircraftCommand(
                                        this.state.selectedAircraftID.toString(),
                                        "ISSUE_COMMAND",
                                        "RTL"
                                    )
                                }
                            />
                        </MuiThemeProvider>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton
                                icon={
                                    <i className="material-icons">
                                        {this.state.missionButtonIcon}
                                    </i>
                                }
                                style={styles.buttonStyle}
                                label={this.state.missionButtonText}
                                onClick={this.handleStartMission}
                            />
                        </MuiThemeProvider>
                    </div>
                )}
            </div>
        )
    }
}
