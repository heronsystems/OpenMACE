import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
// import FlatButton from "material-ui/FlatButton";
import * as React from "react";

import { VehicleMessages } from "../../components/VehicleMessages/VehicleMessages";
import { Vehicle } from "../../util/Vehicle/Vehicle";
import { styles } from "./styles";


type Props = {
    connectedVehicles: { [id: string]: Vehicle };
    minRange: number;
    maxRange: number;
    handleChangeSelectedVehicle: (vehicleID: string) => void;
};

type State = {
};

export class VehicleMessagesHUDs extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
        };
    }

    render() {
        let vehicleMessages: JSX.Element[] = [];
        for (let key in this.props.connectedVehicles) {
            if (key) {
                if (parseInt(key) >= this.props.minRange && parseInt(key) <= this.props.maxRange) {
                    let vehicle = this.props.connectedVehicles[key];
                    let now = new Date();
                    const lastHeardSeconds = (now.getTime() - vehicle.general.lastHeard.getTime()) / 1000; // Time in seconds
                    if (lastHeardSeconds <= 60) {
                        vehicleMessages.push(
                            <VehicleMessages
                                key={key}
                                vehicleID={key}
                                aircraft={vehicle}
                                handleChangeSelectedVehicle={this.props.handleChangeSelectedVehicle}
                            />
                        );
                    }
                }
            }
        }

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div style={styles.vehicleMessagesHUDsContainer}>
                    {vehicleMessages}
                </div>
            </MuiThemeProvider>



        );
    }
}
