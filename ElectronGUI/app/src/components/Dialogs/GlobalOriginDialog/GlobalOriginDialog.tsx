import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import Dialog from "material-ui/Dialog";
import FlatButton from "material-ui/FlatButton";
import TextField from "material-ui/TextField";
import * as React from "react";
import { Col, Grid } from "react-bootstrap";
import * as GlobalTypes from "../../../types/globalTypings";

import * as L from "leaflet";
import { styles } from "./styles";

type Props = {
    open: boolean;
    handleClose: () => void;
    onGlobalHomeCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void;
    globalOrigin: GlobalTypes.PositionType;
    handleSave: (vehicleHome: GlobalTypes.PositionType) => void;
    contextAnchor: L.LeafletMouseEvent;
    useContext: boolean;
};

type State = {
    globalLat?: number;
    globalLon?: number;
    globalAlt?: number;
};

export class GlobalOriginDialog extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
            globalLat: this.props.useContext ? this.props.contextAnchor.latlng.lat : 0,
            globalLon: this.props.useContext ? this.props.contextAnchor.latlng.lng : 0,
            globalAlt: 0
        };
    }

    componentWillReceiveProps(nextProps: Props) {
        if (nextProps.useContext) {
            this.setState({
                globalLat: this.props.contextAnchor.latlng.lat,
                globalLon: this.props.contextAnchor.latlng.lng
            });
        }
    }

    handleTextChange = (event: any) => {
        this.setState({ [event.target.id]: event.target.value });
    };

    handleSave = () => {
        let globalHome: GlobalTypes.PositionType = {
            lat: this.state.globalLat,
            lng: this.state.globalLon,
            alt: this.state.globalAlt
        };
        this.props.handleSave(globalHome);
        this.props.handleClose();
    };

    render() {
        const actions = [
            <FlatButton label="Cancel" onClick={this.props.handleClose} />,
            <FlatButton label="Save" labelStyle={styles.labelStyle} onClick={this.handleSave} />
        ];

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Dialog
                    titleStyle={styles.dialogTitle}
                    title="Set global origin position"
                    actions={actions}
                    modal={false}
                    open={this.props.open}
                    onRequestClose={this.props.handleClose}
                    contentStyle={styles.dialogContent}
                >
                    <Grid fluid>
                        <Col xs={12} md={12}>
                            <TextField
                                id={"globalLat"}
                                floatingLabelText="Latitude (decimal)"
                                floatingLabelFocusStyle={styles.floatingLabelFocus}
                                underlineFocusStyle={styles.underlineFolcusStyle}
                                onChange={this.handleTextChange}
                                value={this.state.globalLat}
                                type={"number"}
                            />
                        </Col>
                        <Col xs={12} md={12}>
                            <TextField
                                id={"globalLon"}
                                floatingLabelText="Longitude (decimal)"
                                floatingLabelFocusStyle={styles.floatingLabelFocus}
                                underlineFocusStyle={styles.underlineFolcusStyle}
                                onChange={this.handleTextChange}
                                value={this.state.globalLon}
                                type={"number"}
                            />
                        </Col>
                        {/* NOT SETTING ALTITUDE AT THIS TIME--DEFAULTED TO 0
                        <Col xs={12}>
                            <TextField
                                floatingLabelText="Altitude"
                                floatingLabelFocusStyle={{color: colors.orange700}}
                                underlineFocusStyle={{borderColor: colors.orange700}}
                                value={this.state.globalAlt}
                            />
                        </Col>
                        */}
                    </Grid>
                </Dialog>
            </MuiThemeProvider>
        );
    }
}
