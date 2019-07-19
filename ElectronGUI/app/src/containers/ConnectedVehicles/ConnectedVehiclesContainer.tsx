import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import FlatButton from "material-ui/FlatButton";
import * as React from "react";

// import { VehicleHUD } from "../../components/VehicleHUD/VehicleHUD";
// import { VehicleMessages } from "../../components/VehicleMessages/VehicleMessages";
import { Vehicle } from "../../util/Vehicle/Vehicle";
import { styles } from "./styles";

import { ConnectedVehicleHUDs } from "../ConnectedVehicleHUDs/ConnectedVehicleHUDs";
import { VehicleMessagesHUDs } from "../VehicleMessagesHUDs/VehicleMessagesHUDs";

import { RangeSelect } from "../../components/RangeSelect/RangeSelect";

import MenuItem from "material-ui/MenuItem";
import SelectField from "material-ui/SelectField";


type Props = {
    connectedVehicles: { [id: string]: Vehicle };
    onAircraftCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void;
    handleChangeSelectedVehicle: (vehicleID: string) => void;
    selectedVehicleID: string;
    onDisconnectedVehicle: (vehicleID: string) => void;
};

type State = {
    showHUDs?: boolean;
    selectedRangeText?: string,
    showRangeSelect?: boolean,
    minRange?: number,
    maxRange?: number,
    rangeSize?: number
};

export class ConnectedVehiclesContainer extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
            showHUDs: true,
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

    // handleTextChange = (event: any) => {
    //     this.setState({ rangeSize: event.target.value });
    // };

    handleDropdownChange = (event: any, index: number, value: string) => {
        this.setState({ rangeSize: parseInt(value) });
    };

    render() {

        // let rangeValues: JSX.Element[] = [];
        // for (let i = 0; i < Object.keys(this.props.connectedVehicles).length; i++) {
        //     rangeValues.push(<MenuItem key={i} value={i} primaryText={i} label={i} />);
        // }

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div>
                    {Object.keys(this.props.connectedVehicles).length > 0 ? (
                        <div style={styles.connectedVehiclesContainer}>
                            <div>
                                <FlatButton
                                    label={this.state.showHUDs ? "Show Vehicle Messages" : "Show Vehicle HUDs"}
                                    labelPosition="after"
                                    onClick={() =>
                                        this.setState({
                                            showHUDs: !this.state.showHUDs
                                        })
                                    }
                                    icon={
                                        this.state.showHUDs ? (
                                            <i className="material-icons">keyboard_arrow_right</i>
                                        ) : (
                                            <i className="material-icons">keyboard_arrow_left</i>
                                        )
                                    }
                                />
                            </div>

                            <div style={styles.rangeSelectStyle}>
                                {Object.keys(this.props.connectedVehicles).length > 1 &&
                                    <SelectField
                                            floatingLabelText="Range"
                                            style={styles.selectField}
                                            value={this.state.rangeSize.toString()}
                                            onChange={this.handleDropdownChange}
                                        >
                                            <MenuItem value={"2"} primaryText={"2"} label={"2"} />
                                            <MenuItem value={"3"} primaryText={"3"} label={"3"} />
                                            <MenuItem value={"4"} primaryText={"4"} label={"4"} />
                                            <MenuItem value={"5"} primaryText={"5"} label={"5"} />
                                    </SelectField>
                                }


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

                            {this.state.showHUDs ?
                                <ConnectedVehicleHUDs
                                    connectedVehicles={this.props.connectedVehicles}
                                    onAircraftCommand={this.handleAircraftCommand}
                                    handleChangeSelectedVehicle={this.props.handleChangeSelectedVehicle}
                                    selectedVehicleID={this.props.selectedVehicleID}
                                    onDisconnectedVehicle={this.props.onDisconnectedVehicle}
                                    minRange={this.state.minRange}
                                    maxRange={this.state.maxRange}
                                />
                            :
                                <VehicleMessagesHUDs
                                    connectedVehicles={this.props.connectedVehicles}
                                    minRange={this.state.minRange}
                                    maxRange={this.state.maxRange}
                                    handleChangeSelectedVehicle={this.props.handleChangeSelectedVehicle}
                                />
                            }
                        </div>
                    ) : null}
                </div>
            </MuiThemeProvider>
        );
    }
}
