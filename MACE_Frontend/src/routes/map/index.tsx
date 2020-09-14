import * as React from "react";
import MapView from "../../components/map";
import Layout from "../../layouts";
import AircraftHUD from "../../components/aircraft-hud";
const { createRef } = React;
import { LatLng } from "leaflet";
import { withAppContext, Context } from "../../Context";
import { checkIfEqual } from "../../util/helpers";
const { ipcRenderer } = window.require("electron");

type Props = {
  context: Context;
};

type State = {
  showHUD?: boolean;
  goHerePts?: L.LatLng[];
};

class MapRoute extends React.Component<Props, State> {
  _map;
  constructor(props) {
    super(props);
    this._map = createRef();
    this.state = {
      showHUD: true,
      goHerePts: []
    };
  }
  componentDidMount() {
    ipcRenderer.on("toggle_hud", () => {
      this.setState({ showHUD: !this.state.showHUD });
    });
  }
  onRequestCenter = (center: LatLng) => {
    this._map.current.setCenter(center);
  };
  shouldComponentUpdate(prevProps: Props) {
    let shouldUpdate = false;
    if (prevProps.context && this.props.context) {
      const { aircrafts: prevAircrafts } = prevProps?.context;
      const { aircrafts: nextAircrafts } = this.props?.context;
      if (prevAircrafts) {
        prevAircrafts.forEach((a) => {
          const matchingAC = nextAircrafts.find((b) => b.agentID === a.agentID);
          if (matchingAC) {
            let equal = checkIfEqual(a, matchingAC, ["location"]);
            if (!equal) {
              shouldUpdate = true;
              return shouldUpdate;
            }
          } else {
            shouldUpdate = true;
            return;
          }
        });
      }
    } else {
      return true;
    }
    return shouldUpdate;
  }

  onUpdateGoHerePts = (pts: L.LatLng[]) => {
      this.setState({goHerePts: pts});
  }

  render() {
    return (
      <Layout>
        <MapView 
            ref={this._map} 
            context={this.props.context} 
            onUpdateGoHerePts={this.onUpdateGoHerePts}
            onCommand={this.props.context.sendToMACE}
        />
        {this.state.showHUD && (
          <AircraftHUD
            onRequestCenter={this.onRequestCenter}
            aircrafts={this.props.context.aircrafts}
            onCommand={this.props.context.sendToMACE}
            defaultAltitude={10}
            goHerePts={this.state.goHerePts}
            onToggleSelect={this.props.context.updateSelectedAircraft}
          />
        )}
      </Layout>
    );
  }
}

export default withAppContext(MapRoute);
