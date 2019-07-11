import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import * as deepcopy from "deepcopy";
import Checkbox from "material-ui/Checkbox";
import Dialog from "material-ui/Dialog";
import FlatButton from "material-ui/FlatButton";
import TextField from "material-ui/TextField";
import * as React from "react";
import { Col, Grid } from "react-bootstrap";
import * as GlobalTypes from "../../types/globalTypings";

import { styles } from "./styles";

type Props = {
    open: boolean;
    handleClose: () => void;
    handleSave: (sliderSettings: GlobalTypes.EnvironmentSettingsType) => void;
    environmentSettings: GlobalTypes.EnvironmentSettingsType;
};

type State = {
    environmentSettings?: GlobalTypes.EnvironmentSettingsType;
    minSliderVal?: number;
    maxSliderVal?: number;
    showBoundingBox?: boolean;
    defaultGridSpacing?: number;
};

export class EnvironmentSettings extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
            environmentSettings: deepcopy(this.props.environmentSettings),
            minSliderVal: this.props.environmentSettings.minSliderVal,
            maxSliderVal: this.props.environmentSettings.maxSliderVal,
            showBoundingBox: this.props.environmentSettings.showBoundingBox,
            defaultGridSpacing: this.props.environmentSettings.gridSpacing
        };
    }

    handleSave = () => {
        let settings: GlobalTypes.EnvironmentSettingsType = {
            minSliderVal: this.state.minSliderVal,
            maxSliderVal: this.state.maxSliderVal,
            showBoundingBox: this.state.showBoundingBox,
            gridSpacing: this.state.defaultGridSpacing
        };
        this.setState({ environmentSettings: settings });
        this.props.handleSave(settings);
        if (settings.gridSpacing >= settings.minSliderVal && settings.gridSpacing <= settings.maxSliderVal) {
            this.props.handleClose();
        }
    };

    handleCancel = () => {
        let tmpPreferences = deepcopy(this.props.environmentSettings);
        this.setState({
            environmentSettings: tmpPreferences,
            minSliderVal: tmpPreferences.minSliderVal,
            maxSliderVal: tmpPreferences.maxSliderVal,
            showBoundingBox: tmpPreferences.showBoundingBox,
            defaultGridSpacing: tmpPreferences.gridSpacing
        });
        this.props.handleClose();
    };

    handleTextChange = (event: any) => {
        this.setState({ [event.target.id]: parseInt(event.target.value) });
    };

    handleCheck = (checked: boolean) => {
        this.setState({ showBoundingBox: checked });
    };

    render() {
        const actions = [
            <FlatButton label="Cancel" onClick={this.handleCancel} />,
            <FlatButton label="Save" labelStyle={styles.labelStyle} onClick={this.handleSave} />
        ];

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Dialog
                    titleStyle={styles.dialogTitle}
                    title="Environment Drawing Settings"
                    actions={actions}
                    modal={false}
                    open={this.props.open}
                    onRequestClose={this.handleCancel}
                >
                    <Grid style={styles.gridStyle} fluid>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <TextField
                                    id={"minSliderVal"}
                                    floatingLabelText="Min Grid Spacing"
                                    floatingLabelFocusStyle={styles.floatingLabelFocus}
                                    underlineFocusStyle={styles.underlineFolcusStyle}
                                    onChange={this.handleTextChange}
                                    type={"number"}
                                    value={this.state.minSliderVal.toString()}
                                />
                            </Col>
                        </Col>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <TextField
                                    id={"maxSliderVal"}
                                    floatingLabelText="Max grid spacing"
                                    floatingLabelFocusStyle={styles.floatingLabelFocus}
                                    underlineFocusStyle={styles.underlineFolcusStyle}
                                    onChange={this.handleTextChange}
                                    type={"number"}
                                    value={this.state.maxSliderVal.toString()}
                                />
                            </Col>
                        </Col>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <TextField
                                    id={"defaultGridSpacing"}
                                    floatingLabelText="Default grid spacing"
                                    floatingLabelFocusStyle={styles.floatingLabelFocus}
                                    underlineFocusStyle={styles.underlineFolcusStyle}
                                    onChange={this.handleTextChange}
                                    type={"number"}
                                    value={this.state.defaultGridSpacing.toString()}
                                />
                            </Col>
                        </Col>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Show bounding box"
                                    labelStyle={styles.checkboxLabelStyle}
                                    style={styles.checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked)}
                                    checked={this.state.showBoundingBox}
                                />
                            </Col>
                        </Col>
                    </Grid>
                </Dialog>
            </MuiThemeProvider>
        );
    }
}
