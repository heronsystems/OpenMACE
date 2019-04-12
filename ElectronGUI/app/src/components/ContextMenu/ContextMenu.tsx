import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import Divider from "material-ui/Divider";
import Menu from "material-ui/Menu";
import MenuItem from "material-ui/MenuItem";
import * as React from "react";
// import { styles } from "./styles";

// import * as colors from 'material-ui/styles/colors';

import * as L from "leaflet";

type Props = {
    selectedVehicleID?: string;
    menuAnchor: L.LeafletMouseEvent;
    handleClose: () => void;
    handleSetHome: () => void;
    handleSetGlobal: () => void;
    handleGoHere: () => void;
    handleSetTakeoff: () => void;
};

type State = {
    xPos?: number;
    yPos?: number;
};

export class ContextMenu extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
            xPos: this.props.menuAnchor ? this.props.menuAnchor.containerPoint.x : 0,
            yPos: this.props.menuAnchor ? this.props.menuAnchor.containerPoint.y : 0
        };
    }

    handleSetHome = () => {
        this.props.handleSetHome();
        this.props.handleClose();
    };

    handleSetGlobal = () => {
        this.props.handleSetGlobal();
        this.props.handleClose();
    };

    handleGoHere = () => {
        this.props.handleGoHere();
        this.props.handleClose();
    };

    handleSetTakeoff = () => {
        this.props.handleSetTakeoff();
        this.props.handleClose();
    };

    render() {
        // TODO: Figure out how to move this style to a separate styles file. The state var is hard:
        const menuStyle = {
            position: "absolute" as "absolute",
            left: this.state.xPos,
            top: this.state.yPos + 65,
            width: 200,
            backgroundColor: "#ffffff"
        };

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Menu style={menuStyle}>
                    <MenuItem primaryText="Set home location" onClick={this.handleSetHome} />
                    <MenuItem primaryText="Set global origin" onClick={this.handleSetGlobal} />
                    <MenuItem primaryText="Set takeoff position" onClick={this.handleSetTakeoff} />

                    <Divider />
                    {this.props.selectedVehicleID !== "0" &&
                        <MenuItem primaryText="'Go-to' here" onClick={this.handleGoHere} />
                    }
                </Menu>
            </MuiThemeProvider>
        );
    }
}
