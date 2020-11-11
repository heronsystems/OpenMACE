import { LatLng } from "leaflet";
import colors from "../../util/colors";
import styles from "./styles";
import * as L from "leaflet";
import * as React from "react";
import { Map, TileLayer, Viewport, Popup } from "react-leaflet";
import { Context as ContextType } from "../../Context";
import ContextMenu from "./components/context-menu";
import Markers from "./components/markers";
import * as Types from "../../data-types/index";

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
  onCommand: (command: string, filteredAircrafts: Types.Aircraft.AircraftPayload[], payload: string[]) => void;
  target: Types.Vertex;
  goHereEnabled: {agentID: string, showGoHere: boolean}[];
};

type State = {
  center?: LatLng;
  zoom?: number;
  contextMenuPosition?: { x: number; y: number };
  contextMenuVisible?: boolean;
  popupVisible?:boolean;
  originPosition?: L.LatLng;
  selectedAircraft?: string;
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
        popupVisible: false,
        originPosition: new LatLng(0,0),
        selectedAircraft: null
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
    if(this.state.popupVisible != nextState.popupVisible) {
      return true;
    }
    if(nextProps.goHereEnabled != this.props.goHereEnabled) {
        return true;
    }
    return false;
  }

  togglePopup = () => {
    this.setState({selectedAircraft: "ALL"});
    this.setState({ popupVisible: !this.state.popupVisible });
  }

  onAircraftSelected = (e) => {
    this.setState({selectedAircraft: e.target.value });
  }
  createDropdownList = () => {
    let options = [];
    let hide = false;
    options.push(<option key={0} value="ALL">All Aircraft</option>);
    this.props.context.aircrafts.forEach((a) => {
      if (["STABILIZE",""].indexOf(a.mode) != -1){
        hide = true;
      } else {
        hide = false;
      }
      options.push(<option key={options.length} value={a.agentID} disabled={hide}>{"Agent " + a.agentID}</option>);
    });
    return options;
  }
  handlevehicleHome = () => {
    this.setState({ popupVisible: false });
    let command: string = "SET_VEHICLE_HOME";
    let payload = {
      lat: this.state.originPosition.lat,
      lng: this.state.originPosition.lng,
      alt: 0.0
    };
    let payloadArray = [];
    let enabledAircraft = [];
    this.props.context.aircrafts.forEach((a) => {
      if (["STABILIZE",""].indexOf(a.mode) == -1){
        enabledAircraft.push(a);
        payloadArray.push(JSON.stringify(payload));
      }
      if (this.state.selectedAircraft === a.agentID) {
        this.props.onCommand(command, [a], [JSON.stringify(payload)]);
      }
    });
    if (this.state.selectedAircraft === "ALL") {
      this.props.onCommand(command, enabledAircraft, payloadArray);
    }
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
    this.setState({popupVisible: false});
  }

  removeContextMenu = () => {
    this.setState({contextMenuVisible: false});
  }

  setSwarmOrigin = () => {
    //   console.log("Set origin to: ");
    //   console.log(this.state.originPosition);
      let command: string = "SET_SWARM_ORIGIN";
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
        preferCanvas={true}
      >
        <TileLayer
          url="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
          attribution="Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community"
        />

        <Markers
            goHereEnabled={this.props.goHereEnabled}
         />

        <ContextMenu
          position={this.state.contextMenuPosition}
          visible={this.state.contextMenuVisible}
          onRequestClose={() => this.removeContextMenu()}
          actions={[{ label: "Set global origin", action: this.setSwarmOrigin }, { label: "Set Vehicle origin", action: this.togglePopup }]}
        />
        {this.state.popupVisible &&
          <Popup
            position={[this.state.originPosition.lat, this.state.originPosition.lng]}
            autoClose={false}
            closeOnClick={true}
            closeButton={false}
            offset={[10, 15]}
          >
            <span style={styles.inputLabel}>Select Aircraft:</span>
            <div>
              <select style={styles.selectRow} onChange={this.onAircraftSelected} >
                {this.createDropdownList()}
              </select>
            </div>

            <div style={styles.singleSettingContainer}>
              <span style={styles.inputLabel}>Latitude:</span>
              <input
                id="latitude-input"
                type="number"
                value={this.state.originPosition.lat}
                onChange={(e) => {
                  const { name, value } = e.target;
                }}
                name={"lat"}
                style={styles.input}
              />
              <span style={styles.inputLabel}>Longitude:</span>
              <input
                id="longitude-input"
                type="number"
                value={this.state.originPosition.lng}
                onChange={(e) => {
                  const { name, value } = e.target;
                }}
                name={"lng"}
                style={styles.input}
              />
            </div>

            <div style={styles.actionsContainer}>
              <button style={styles.cancelButton} onClick={this.togglePopup}>
                Cancel
              </button>
              <button style={styles.saveButton} onClick={this.handlevehicleHome}>
                Save
              </button>
            </div>
          </Popup>
        }


      </Map>
    );
  }
}
