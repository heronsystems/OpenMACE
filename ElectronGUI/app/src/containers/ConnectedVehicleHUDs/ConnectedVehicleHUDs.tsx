import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
// import FlatButton from "material-ui/FlatButton";
import * as React from "react";

import { VehicleHUD } from "../../components/VehicleHUD/VehicleHUD";
// import { Card } from "material-ui/Card";
import { Vehicle } from "../../util/Vehicle/Vehicle";
import { styles } from "./styles";
import { orange400 } from "material-ui/styles/colors";

import * as GlobalTypes from "../../types/globalTypings";

type Props = {
    connectedVehicles: { [id: string]: Vehicle };
    onAircraftCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void;
    handleChangeSelectedVehicle: (vehicleID: string) => void;
    selectedVehicleID: string;
    onDisconnectedVehicle: (vehicleID: string) => void;
    minRange: number;
    maxRange: number;
};

type State = {
};

export class ConnectedVehicleHUDs extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
        };
    }

    handleAircraftCommand = (vehicleID: string, tcpCommand: string, vehicleCommand: string) => {
        console.log("Command: " + vehicleCommand + " for vehicleID: " + vehicleID);
        this.props.onAircraftCommand(vehicleID, tcpCommand, vehicleCommand);
    };

    render() {

        // let vehicleHUDs: JSX.Element[] = [];
        // for (let key in this.props.connectedVehicles) {
        //     if (key) {
        //         if (parseInt(key) >= this.props.minRange && parseInt(key) <= this.props.maxRange) {
        //             let vehicle = this.props.connectedVehicles[key];
        //             let now = new Date();
        //             const lastHeardSeconds = (now.getTime() - vehicle.general.lastHeard.getTime()) / 1000; // Time in seconds
        //             if (lastHeardSeconds <= 60) {
        //                 vehicleHUDs.push(
        //                     <VehicleHUD
        //                         key={key}
        //                         vehicleID={key}
        //                         aircraft={vehicle}
        //                         handleAircraftCommand={this.handleAircraftCommand}
        //                         handleChangeSelectedVehicle={this.props.handleChangeSelectedVehicle}
        //                         highlightColor={this.props.connectedVehicles[key].highlightColor}
        //                     />
        //                 );

        //             } else {
        //                 this.props.onDisconnectedVehicle(key);
        //             }
        //         }
        //     }
        // }

        let vehicleHUDs: JSX.Element[] = [];
        for (let i = 0; i < 10; i ++) {
            let position: GlobalTypes.PositionType = {
                alt: i,
                lat: i,
                lng: i
            };
            let attitude: GlobalTypes.AttitudeType = {
                roll: i,
                pitch: i,
                yaw: i
            };
            vehicleHUDs.push(
                <VehicleHUD
                    key={i}
                    vehicleID={i.toString()}
                    aircraft={new Vehicle(i, position, attitude)}
                    handleAircraftCommand={this.handleAircraftCommand}
                    handleChangeSelectedVehicle={this.props.handleChangeSelectedVehicle}
                    highlightColor={orange400}
                />
            )
        }

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div style={styles.connectedVehicleHUDsContainer}>
                    {vehicleHUDs}
                </div>
            </MuiThemeProvider>



        );
    }
}
