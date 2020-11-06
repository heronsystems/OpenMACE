import * as React from "react";
import MapView from "../../components/map";
import Layout from "../../layouts";
import AircraftHUD from "../../components/aircraft-hud";
const { createRef } = React;
import { LatLng } from "leaflet";
import { withAppContext, Context } from "../../Context";
import { checkIfEqual } from "../../util/helpers";
import { Vertex } from "../../data-types";
import { cloneDeep } from "lodash";
const { ipcRenderer } = window.require("electron");

type Props = {
  context: Context;
};

type State = {
  showHUD?: boolean;
  showTarget?: { agentID: string; showGoHere: boolean }[];
  goHerePt?: Vertex;
};

class MapRoute extends React.Component<Props, State> {
  _map;
  constructor(props) {
    super(props);
    this._map = createRef();
    this.state = {
      showHUD: true,
      showTarget: [],
      goHerePt: {
        lat: 0,
        lng: 0,
        alt: 0,
      },
    };
  }
  componentDidMount() {
    ipcRenderer.on("toggle_hud", () => {
      this.setState({ showHUD: !this.state.showHUD });
    });
  }
  onRequestCenter = (agentID: string) => {
    const agent = this.props.context.aircrafts.find(
      (a) => a.agentID === agentID
    );
    const { lat, lng } = agent.location;
    this._map.current.setCenter({ lat, lng });
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
    // console.log("MAP-UPDATE GO HERE>");
    let a = {
      lat: pts.lat,
      lng: pts.lng,
      alt: this.state.goHerePt.alt,
    };
    this.setState({ goHerePt: a });
    this.updateGoHerePt();
  };

  HUDUpdateGoHerePt = (point: Vertex & { agentID: string }) => {
    // console.log("HUD-UPDATE GO HERE");
    this.setState({ goHerePt: point });
    this.updateGoHerePt(point.agentID);
  };

  toggleGoHerePt = (show: boolean, agentID: string) => {
    //   console.log("TOGGLE GO HERE");
    //   console.log(show);
    let goHereVec = cloneDeep(this.state.showTarget);
    if (
      goHereVec.filter(function (p) {
        return p.agentID === agentID;
      }).length > 0
    ) {
      goHereVec.forEach((element) => {
        if (element.agentID === agentID) {
          element.showGoHere = show;
        }
      });
    } else {
      goHereVec.push({ agentID: agentID, showGoHere: show });
    }

    this.setState({ showTarget: goHereVec }, () =>
      this.updateGoHerePt(agentID)
    );
  };

  updateGoHerePt = (agentID?: string) => {
    if (!agentID) {
      this.state.showTarget.forEach((element) => {
        if (element.showGoHere) {
          agentID = element.agentID;
        }
      });
    }

    let display = this.state.showTarget.filter(function (p) {
      return p.agentID === agentID;
    });

    this.props.context.updateTargets({
      location: { lat: this.state.goHerePt.lat, lng: this.state.goHerePt.lng },
      is_global: true,
      should_display: display[0] ? display[0].showGoHere : false,
      distance_to_target: 0.0,
      agentID: agentID,
    });
  };
  render() {
    return (
      <Layout>
        <MapView
          ref={this._map}
          context={this.props.context}
          onUpdateGoHerePts={this.mapUpdateGoHerePt}
          target={this.state.goHerePt}
          onCommand={this.props.context.sendToMACE}
          goHereEnabled={this.state.showTarget}
        />
        {this.state.showHUD && (
          <AircraftHUD
            onRequestCenter={this.onRequestCenter}
            aircrafts={this.props.context.aircrafts}
            onCommand={this.props.context.sendToMACE}
            onUpdateGoHerePts={this.HUDUpdateGoHerePt}
            toggleGoHerePt = {this.toggleGoHerePt}
            target={this.state.goHerePt}
            defaultAltitude={50}
            onToggleSelect={this.props.context.updateSelectedAircraft}
            addNotification={this.props.context.addNotification}
            showTargetFlags={this.state.showTarget}
          />
        )}
      </Layout>
    );
  }
}

export default withAppContext(MapRoute);
