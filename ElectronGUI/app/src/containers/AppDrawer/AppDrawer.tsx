import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import Divider from "material-ui/Divider";
import Drawer from "material-ui/Drawer";
import FlatButton from "material-ui/FlatButton";
import FontIcon from "material-ui/FontIcon";
import MenuItem from "material-ui/MenuItem";
import * as React from "react";
import { styles } from "./styles";

type Props = {
    openDrawer: boolean;
    onToggleDrawer: (open: boolean) => void;
    onDrawerAction: (action: string) => void;
    showMessagesMenu: boolean;
};

type State = {
    openSettingsSubmenu?: boolean;
};

export class AppDrawer extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
            openSettingsSubmenu: false
        };
    }

    render() {
        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Drawer
                    containerStyle={styles.drawerContainer}
                    docked={true}
                    open={this.props.openDrawer}
                    onRequestChange={(open: boolean) => this.props.onToggleDrawer(open)}
                >
                    <div style={styles.logo}>
                        <img className="image" src={"images/HeronLogo_Small.png"} />
                    </div>

                    <Divider />

                    <MenuItem onClick={() => this.props.onDrawerAction("TestButton1")}>
                        <FlatButton
                            labelStyle={styles.drawerItemLabelStyle}
                            label="TESTING 1"
                            icon={<FontIcon className="material-icons">gesture</FontIcon>}
                            disableTouchRipple={false}
                            disableFocusRipple={false}
                            hoverColor={"rgba(0,0,0, 0.0)"}
                        />
                    </MenuItem>

                    <MenuItem onClick={() => this.props.onDrawerAction("TestButton2")}>
                        <FlatButton
                            labelStyle={styles.drawerItemLabelStyle}
                            label="TESTING 2"
                            icon={<FontIcon className="material-icons">gesture</FontIcon>}
                            disableTouchRipple={false}
                            disableFocusRipple={false}
                            hoverColor={"rgba(0,0,0, 0.0)"}
                        />
                    </MenuItem>

                    <MenuItem onClick={() => this.props.onDrawerAction("EditEnvironment")}>
                        <FlatButton
                            labelStyle={styles.drawerItemLabelStyle}
                            label="Environment"
                            icon={<FontIcon className="material-icons">border_style</FontIcon>}
                            disableTouchRipple={false}
                            disableFocusRipple={false}
                            hoverColor={"rgba(0,0,0, 0.0)"}
                        />
                    </MenuItem>

                    {/*
                    <MenuItem onClick={() => this.props.onDrawerAction("Logging")}>
                        <FlatButton
                        labelStyle={drawerItemLabelStyle}
                        label="Logging"
                        icon={<FontIcon className="material-icons">event_note</FontIcon>}
                        />
                    </MenuItem>
                    */}

                    <Divider />

                    <MenuItem
                        onClick={() =>
                            this.setState({
                                openSettingsSubmenu: !this.state.openSettingsSubmenu
                            })
                        }
                    >
                        <FlatButton
                            labelStyle={styles.drawerItemLabelStyle}
                            label="Settings"
                            labelPosition="before"
                            icon={
                                this.state.openSettingsSubmenu ? (
                                    <FontIcon className="material-icons">expand_less</FontIcon>
                                ) : (
                                    <FontIcon className="material-icons">expand_more</FontIcon>
                                )
                            }
                            disableTouchRipple={false}
                            disableFocusRipple={false}
                            hoverColor={"rgba(0,0,0, 0.0)"}
                        />
                        {this.state.openSettingsSubmenu && (
                            <div>
                                <MenuItem onClick={() => this.props.onDrawerAction("MACEConfig")}>
                                    <FlatButton
                                        labelStyle={styles.drawerItemLabelStyle}
                                        label="MACE Config"
                                        icon={<FontIcon className="material-icons">flight_takeoff</FontIcon>}
                                        disableTouchRipple={false}
                                        disableFocusRipple={false}
                                        hoverColor={"rgba(0,0,0, 0.0)"}
                                    />
                                </MenuItem>
                                <MenuItem onClick={() => this.props.onDrawerAction("Messages")}>
                                    <FlatButton
                                        labelStyle={styles.drawerItemLabelStyle}
                                        label="Messages"
                                        icon={<FontIcon className="material-icons">event_note</FontIcon>}
                                        disableTouchRipple={false}
                                        disableFocusRipple={false}
                                        hoverColor={"rgba(0,0,0, 0.0)"}
                                    />
                                </MenuItem>
                                <MenuItem onClick={() => this.props.onDrawerAction("Takeoff")}>
                                    <FlatButton
                                        labelStyle={styles.drawerItemLabelStyle}
                                        label="Takeoff"
                                        icon={<FontIcon className="material-icons">flight_takeoff</FontIcon>}
                                        disableTouchRipple={false}
                                        disableFocusRipple={false}
                                        hoverColor={"rgba(0,0,0, 0.0)"}
                                    />
                                </MenuItem>
                            </div>
                        )}
                    </MenuItem>
                </Drawer>
            </MuiThemeProvider>
        );
    }
}
