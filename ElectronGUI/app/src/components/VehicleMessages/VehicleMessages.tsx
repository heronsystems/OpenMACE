import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import Avatar from "material-ui/Avatar";
import { Card, CardActions, CardHeader, CardText } from "material-ui/Card";
import FlatButton from "material-ui/FlatButton";
import * as React from "react";
import { textSeverityToColor } from "../../util/misc/Colors";
import { Colors } from "../../util/misc/Colors";
import { aircraftImgSrcFromType } from "../../util/Helpers/VehicleHelper";
import { Vehicle } from "../../util/Vehicle/Vehicle";
import { styles } from "./styles";

type Props = {
    vehicleID: string;
    aircraft: Vehicle;
};

type State = {};

export class VehicleMessages extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);
    }

    handleClear = () => {
        this.props.aircraft.messages = [];
    };

    render() {
        // TODO: Figure out how to move this style to a separate styles file. The conditional is hard:
        const boxShadow = this.props.aircraft.isSelected
            ? this.props.aircraft.highlightColor + " 0px 1px 20px, rgba(0, 0, 0, .5) 0px 1px 4px"
            : "rgba(0, 0, 0, 0.117647) 0px 1px 6px, rgba(0, 0, 0, 0.117647) 0px 1px 4px";
        const hudStyle = {
            position: "relative" as "relative",
            width: 90 + "%",
            marginBottom: 15,
            boxShadow: boxShadow
        };

        // const hudAvatar = aircraftImgSrcFromType(this.props.aircraft.general.aircraftType);
        const hudAvatar: JSX.Element = (
            <Avatar
                backgroundColor={this.props.aircraft.isArmed ? Colors.Success : Colors.Primary}
                src={aircraftImgSrcFromType(this.props.aircraft.general.aircraftType)}
                style={styles.avatar}
            />
        );

        let messages: JSX.Element[] = [];
        for (let key in this.props.aircraft.messages) {
            if (key) {
                let textColor = textSeverityToColor(this.props.aircraft.messages[key].severity);
                messages.push(
                    <CardText
                        key={key}
                        style={{
                            color: textColor,
                            fontSize: 18,
                            paddingTop: 0,
                            paddingBottom: 0
                        }}
                    >
                        {"[" + this.props.aircraft.messages[key].severity + "] : " + this.props.aircraft.messages[key].text}
                    </CardText>
                );
            }
        }

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Card expanded={true} style={hudStyle}>
                    <CardHeader
                        titleStyle={styles.cardTitle}
                        title={"ID: " + this.props.vehicleID}
                        subtitle={this.props.aircraft.vehicleMode}
                        avatar={hudAvatar}
                        actAsExpander={true}
                        showExpandableButton={false}
                    />

                    <div className="row">
                        <div className="col-xs-12">
                            <div className="box" style={styles.messagesContainer}>
                                {messages}
                            </div>
                        </div>
                    </div>

                    <CardActions style={styles.cardActions}>
                        <FlatButton label="Clear" onClick={this.handleClear} icon={<i className="material-icons">delete_sweep</i>} />
                    </CardActions>
                </Card>
            </MuiThemeProvider>
        );
    }
}
