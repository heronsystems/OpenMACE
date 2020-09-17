import { LatLng } from "leaflet";
import * as React from "react";
import { Map, TileLayer, Viewport } from "react-leaflet";
import { Context as ContextType } from "../../Context";
import ContextMenu from "./components/context-menu";
import Markers from "./components/markers";
import { Vertex } from "../../data-types";
import DefaultMarker from "./components/default-marker";
const { createRef } = React;
const { ipcRenderer } = window.require("electron");

const DEFAULT_CENTER: LatLng = new LatLng(-35.361196112174795, 149.15725708007815);
// const DEFAULT_CENTER: LatLng = new LatLng(
//   38.28454701883166,
//   -76.66688919067384
// );
const DEFAULT_ZOOM = 14;

type Props = {
  context?: ContextType;
  onUpdateGoHerePts: (pts: L.LatLng) => void;
  onCommand: (command: string, filteredAircrafts: Aircraft.AircraftPayload[], payload: string[]) => void;
  target: Vertex;
};

type State = {
  center?: LatLng;
  zoom?: number;
  contextMenuPosition?: { x: number; y: number };
  contextMenuVisible?: boolean;
  originPosition?: L.LatLng;
};

// Used for debugging. Log clicked points on map for reference
const pts = [];

export default class MapView extends React.Component<Props, State> {
  _map: React.RefObject<Map> = createRef();
  constructor(props) {
    super(props);
    this.state = {
        contextMenuPosition: { x: 0, y: 0 },
        contextMenuVisible: false,
        originPosition: new LatLng(0,0)
    };
  }
  setZoom = (e) => {
    this.props.context.setGlobalZoom(e.target.getZoom());
  };
  componentDidMount() {
    const { updateIcons } = this.props.context;
    ipcRenderer.on("takeoff_land", () => {
      updateIcons({
        name: "test-takeoff",
        type: "takeoff_land",
        lat: this._map.current.leafletElement.getCenter().lat,
        lng: this._map.current.leafletElement.getCenter().lng,
        auto_focus: true
      });
      // TODO: Send a message back to the server
    });
  }
  handleViewportChange = (viewport: Viewport) => {};
  setCenter = (center: LatLng) => {
    this._map.current.leafletElement.panTo(center);
  };
  shouldComponentUpdate(nextProps: Props, nextState: State) {
    if(this.state.contextMenuVisible != nextState.contextMenuVisible) {
        return true;
    }
    return false;
  }

  handleContextMenu = (e: L.LeafletMouseEvent) => {
    this.setState({
        contextMenuPosition: {
            x: e.originalEvent.clientX,
            y: e.originalEvent.clientY
        },
        originPosition: e.latlng
    });
    
    this.setState({contextMenuVisible: true});
  }

  removeContextMenu = () => {
    this.setState({contextMenuVisible: false});
  }

  setGlobalOrigin = () => {
    //   console.log("Set origin to: ");
    //   console.log(this.state.originPosition);
      let command: string = "SET_GLOBAL_ORIGIN";
      let payload = {
          lat: this.state.originPosition.lat,
          lng: this.state.originPosition.lng,
          alt: 0.0
      };
      this.props.onCommand(command, [], [JSON.stringify(payload)]);
  }

  render() {
    return (
      <Map
        center={DEFAULT_CENTER}
        zoom={DEFAULT_ZOOM}
        id="map"
        zoomControl={false}
        onViewportChange={this.handleViewportChange}
        ref={this._map}
        /// @ts-ignore This does exist, TS is being dumb
        onClick={(e) => {
          this.props.onUpdateGoHerePts(e.latlng);
        }}
        minZoom={5}
        maxZoom={18}
        onZoomEnd={this.setZoom}
        animate={true}
        oncontextmenu={this.handleContextMenu}
      >
        <TileLayer
          url="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
          attribution="Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community"
        />

        <Markers />
        
        <ContextMenu
          position={this.state.contextMenuPosition}
          visible={this.state.contextMenuVisible}
          onRequestClose={() => this.removeContextMenu()}
          actions={[{ label: "Set global origin", action: this.setGlobalOrigin }]}
        />
      </Map>
    );
  }
}
