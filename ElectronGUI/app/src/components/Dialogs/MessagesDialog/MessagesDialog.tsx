import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import * as deepcopy from "deepcopy";
import Checkbox from "material-ui/Checkbox";
import Dialog from "material-ui/Dialog";
import FlatButton from "material-ui/FlatButton";
import * as React from "react";
import { Col, Grid } from "react-bootstrap";
import * as GlobalTypes from "../../../types/globalTypings";

import { styles } from "./styles";

type Props = {
    open: boolean;
    handleClose: () => void;
    handleSave: (preferences: GlobalTypes.MessagePreferencesType) => void;
    preferences: GlobalTypes.MessagePreferencesType;
};

type State = {
    preferences?: GlobalTypes.MessagePreferencesType;
};

export class MessagesDialog extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
            preferences: deepcopy(this.props.preferences)
        };
    }

    handleSave = () => {
        this.props.handleSave(this.state.preferences);
        this.props.handleClose();
    };

    handleCheck = (checked: boolean, preference: string) => {
        let stateVar: any = deepcopy(this.state.preferences);
        stateVar[preference] = checked;
        this.setState({ preferences: stateVar });
    };

    handleCancel = () => {
        let tmpPreferences = deepcopy(this.props.preferences);
        this.setState({ preferences: tmpPreferences });
        this.props.handleClose();
    };

    render() {
        const actions = [
            <FlatButton label="Cancel" onTouchTap={this.handleCancel} />,
            <FlatButton label="Save" labelStyle={styles.labelStyle} onTouchTap={this.handleSave} />
        ];

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Dialog
                    titleStyle={styles.dialogTitle}
                    title="Message Display Preferences"
                    actions={actions}
                    modal={false}
                    open={this.props.open}
                    onRequestClose={this.handleCancel}
                >
                    <Grid style={styles.gridStyle} fluid>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Emergency"
                                    labelStyle={styles.checkboxLabel_Emergency}
                                    style={styles.checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "emergency")}
                                    checked={this.state.preferences.emergency}
                                />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Critical"
                                    labelStyle={styles.checkboxLabel_Critical}
                                    style={styles.checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "critical")}
                                    checked={this.state.preferences.critical}
                                />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Alert"
                                    labelStyle={styles.checkboxLabel_Alert}
                                    style={styles.checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "alert")}
                                    checked={this.state.preferences.alert}
                                />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Error"
                                    labelStyle={styles.checkboxLabel_Error}
                                    style={styles.checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "error")}
                                    checked={this.state.preferences.error}
                                />
                            </Col>
                        </Col>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Warning"
                                    labelStyle={styles.checkboxLabel_Warning}
                                    style={styles.checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "warning")}
                                    checked={this.state.preferences.warning}
                                />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Notice"
                                    labelStyle={styles.checkboxLabel_Notice}
                                    style={styles.checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "notice")}
                                    checked={this.state.preferences.notice}
                                />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Info"
                                    labelStyle={styles.checkboxLabel_Info}
                                    style={styles.checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "info")}
                                    checked={this.state.preferences.info}
                                />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Debug"
                                    labelStyle={styles.checkboxLabel_Debug}
                                    style={styles.checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "debug")}
                                    checked={this.state.preferences.debug}
                                />
                            </Col>
                        </Col>
                    </Grid>
                </Dialog>
            </MuiThemeProvider>
        );
    }
}
