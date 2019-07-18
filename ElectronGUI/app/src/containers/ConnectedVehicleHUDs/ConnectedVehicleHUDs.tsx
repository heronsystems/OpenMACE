import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import FlatButton from "material-ui/FlatButton";
import * as React from "react";

import { VehicleHUD } from "../../components/VehicleHUD/VehicleHUD";
import { Vehicle } from "../../util/Vehicle/Vehicle";
import { styles } from "./styles";

import { RangeSelect } from "../../components/RangeSelect/RangeSelect";

type Props = {
    connectedVehicles: { [id: string]: Vehicle };
    onAircraftCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void;
    handleChangeSelectedVehicle: (vehicleID: string) => void;
    selectedVehicleID: string;
    onDisconnectedVehicle: (vehicleID: string) => void;
};

type State = {
    selectedRangeText?: string,
    showRangeSelect?: boolean,
    minRange?: number,
    maxRange?: number,
    rangeSize?: number
};

export class ConnectedVehicleHUDs extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
            selectedRangeText: "1 - 3",
            showRangeSelect: false,
            minRange: 1,
            maxRange: 3,
            rangeSize: 3
        };
    }

    handleAircraftCommand = (vehicleID: string, tcpCommand: string, vehicleCommand: string) => {
        console.log("Command: " + vehicleCommand + " for vehicleID: " + vehicleID);
        this.props.onAircraftCommand(vehicleID, tcpCommand, vehicleCommand);
    };

    decrementSelectedRange = () => {
        if (this.state.minRange !== 1) {
            let tmpMinRange = this.state.minRange - this.state.rangeSize;
            let tmpMaxRange = this.state.maxRange - this.state.rangeSize;
            this.setState({
                minRange: tmpMinRange,
                maxRange: tmpMaxRange,
                selectedRangeText: tmpMinRange + " - " + tmpMaxRange
            });
        }
    };

    incrementSelectedRange = () => {
        console.log(this.state.maxRange + this.state.rangeSize);
        console.log(Object.keys(this.props.connectedVehicles).length);

        if (this.state.minRange + this.state.rangeSize < Object.keys(this.props.connectedVehicles).length) {
            let tmpMinRange = this.state.minRange + this.state.rangeSize;
            let tmpMaxRange = this.state.maxRange + this.state.rangeSize;
            this.setState({
                minRange: tmpMinRange,
                maxRange: tmpMaxRange,
                selectedRangeText: tmpMinRange + " - " + tmpMaxRange
            });
        }
    };

    handleSelectRange = (min: number, max: number) => {
        this.setState({
            minRange: min,
            maxRange: max,
            selectedRangeText: min + " - " + max,
            showRangeSelect: false
        });
    }

    render() {

        let vehicleHUDs: JSX.Element[] = [];
        for (let key in this.props.connectedVehicles) {
            if (key) {
                if (parseInt(key) >= this.state.minRange && parseInt(key) <= this.state.maxRange) {
                    let vehicle = this.props.connectedVehicles[key];
                    let now = new Date();
                    const lastHeardSeconds = (now.getTime() - vehicle.general.lastHeard.getTime()) / 1000; // Time in seconds
                    if (lastHeardSeconds <= 60) {
                        vehicleHUDs.push(
                            <VehicleHUD
                                key={key}
                                vehicleID={key}
                                aircraft={vehicle}
                                handleAircraftCommand={this.handleAircraftCommand}
                                handleChangeSelectedVehicle={this.props.handleChangeSelectedVehicle}
                                highlightColor={this.props.connectedVehicles[key].highlightColor}
                            />
                        );

                    } else {
                        this.props.onDisconnectedVehicle(key);
                    }
                }
            }
        }

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div style={styles.connectedVehicleHUDsContainer}>
                    <FlatButton
                        onClick={this.decrementSelectedRange}
                        icon={<i className="material-icons">keyboard_arrow_left</i>}
                    />
                    <FlatButton
                        label={this.state.selectedRangeText}
                        onClick={() =>
                            this.setState({
                                showRangeSelect: !this.state.showRangeSelect
                            })
                        }
                    />
                    <FlatButton
                        onClick={this.incrementSelectedRange}
                        icon={<i className="material-icons">keyboard_arrow_right</i>}
                    />

                    {this.state.showRangeSelect ?
                        <RangeSelect
                            numVehicles={Object.keys(this.props.connectedVehicles).length}
                            onSelectRange={this.handleSelectRange}
                            selectedMin={this.state.minRange}
                            selectedMax={this.state.maxRange}
                            rangeSize={this.state.rangeSize}
                        />
                    :
                        null
                    }

                </div>
            </MuiThemeProvider>
        );
    }
}
