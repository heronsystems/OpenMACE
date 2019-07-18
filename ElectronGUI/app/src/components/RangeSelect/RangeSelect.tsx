import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();
import * as React from "react";

import FlatButton from "material-ui/FlatButton";
import { Col, Grid } from "react-bootstrap";
import { styles } from "./styles";

import { black, orange700 } from "material-ui/styles/colors";

type Props = {
    numVehicles: number,
    selectedMin: number,
    selectedMax: number,
    rangeSize: number,
    onSelectRange: (min: number, max: number) => void;
};

type State = {
    minRange?: number,
    maxRange?: number
};

export class RangeSelect extends React.Component<Props, State> {
    constructor(props: Props) {
        super(props);

        this.state = {
            minRange: this.props.selectedMin,
            maxRange: this.props.selectedMax
        };
    }

    handleSelectRange = (min: number, max: number) => {
        this.setState({
            minRange: min,
            maxRange: max
        });
        this.props.onSelectRange(min, max);
    }

    render() {
        let ranges: JSX.Element[] = [];
        for (let i = 0; i < this.props.numVehicles; i++) {
            if (i % this.props.rangeSize === this.props.rangeSize || i % this.props.rangeSize === 0) {
                let tmpMin = i + 1;
                let tmpMax = i + this.props.rangeSize;
                let tmpLabel = tmpMin + " - " + tmpMax;
                let labelColor = tmpMin === this.props.selectedMin ? orange700 : black;
                ranges.push(
                    <Col xs={4} md={4}>
                        <FlatButton
                            label={tmpLabel}
                            onClick={() => this.handleSelectRange(tmpMin, tmpMax)}
                            labelStyle={{color: labelColor}}
                        />
                    </Col>
                );
            }
        }

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div style={styles.rangeSelect}>
                    <Grid fluid>
                        <Col xs={12} md={12}>
                            {ranges}
                        </Col>
                    </Grid>
                </div>
            </MuiThemeProvider>
        );
    }
}
