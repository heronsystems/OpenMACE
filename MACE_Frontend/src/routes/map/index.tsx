import * as React from "react";
import MapView from "../../components/map";
import Layout from "../../layouts";
import AircraftHUD from "../../components/aircraft-hud";
const { createRef } = React;
import { LatLng } from "leaflet";
import { withAppContext, Context } from "../../Context";
import { checkIfEqual } from "../../util/helpers";
import { Vertex } from "../../data-types";
const { ipcRenderer } = window.require("electron");

type Props = {
  context: Context;
};

type State = {
  showHUD?: boolean;
  showTarget?: boolean;
  goHerePt?: Vertex;
};

class MapRoute extends React.Component<Props, State> {
  _map;
  constructor(props) {
    super(props);
    this._map = createRef();
    this.state = {
      showHUD: true,
      showTarget: false,
      goHerePt: {
        lat: 0,
        lng: 0,
        alt: 0
      }
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

  mapUpdateGoHerePt = (pts: L.LatLng) => {
    let a = {
      lat: pts.lat,
      lng: pts.lng,
      alt: this.state.goHerePt.alt
    };
    this.setState({goHerePt: a});
    this.sendGoHerePt();
  }

  HUDUpdateGoHerePt = (point: Vertex) => {
    this.setState({goHerePt: point});
    this.sendGoHerePt();
  }

  toggleGoHerePt = (show: boolean) => {
    this.setState({showTarget: show},this.sendGoHerePt);
  }

  sendGoHerePt = () => {
    this.props.context.updateTargets({
      location: { lat: this.state.goHerePt.lat, lng: this.state.goHerePt.lng },
      is_global: true,
      should_display: this.state.showTarget
    })
  }
  render() {
    return (
      <Layout>
        <MapView 
            ref={this._map} 
            context={this.props.context} 
            onUpdateGoHerePts={this.mapUpdateGoHerePt}
            target={this.state.goHerePt}
            onCommand={this.props.context.sendToMACE}
        />
        {this.state.showHUD && (
          <AircraftHUD
            onRequestCenter={this.onRequestCenter}
            aircrafts={this.props.context.aircrafts}
            onCommand={this.props.context.sendToMACE}
            onUpdateGoHerePts={this.HUDUpdateGoHerePt}
            toggleGoHerePt = {this.toggleGoHerePt}
            target={this.state.goHerePt}
            defaultAltitude={10}
            onToggleSelect={this.props.context.updateSelectedAircraft}
          />
        )}
      </Layout>
    );
  }
}

export default withAppContext(MapRoute);
