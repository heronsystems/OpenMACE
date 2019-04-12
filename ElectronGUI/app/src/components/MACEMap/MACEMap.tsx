import * as React from "react";

import getMuiTheme from "material-ui/styles/getMuiTheme";
import MuiThemeProvider from "material-ui/styles/MuiThemeProvider";
const lightMuiTheme = getMuiTheme();

import * as L from "leaflet";
import * as colors from "material-ui/styles/colors";
import { LayerGroup, Map, Marker, Polygon, Polyline, TileLayer } from "react-leaflet";
import { ContextMenu } from "../../components/ContextMenu/ContextMenu";
import * as GlobalTypes from "../../types/globalTypings";
import { Vehicle } from "../../util/Vehicle/Vehicle";
import { Heatmap } from "../mapLayers/heatmap";
import { styles } from "./styles";

type Props = {
    handleSelectedAircraftUpdate: (id: string) => void;
    connectedVehicles: { [id: string]: Vehicle };
    selectedVehicleID: string;
    maxZoom: number;
    mapZoom: number;
    mapCenter: GlobalTypes.PositionType;
    globalOrigin: GlobalTypes.PositionType;
    updateMapCenter: (e: L.DragEndEvent) => void;
    contextSetHome: () => void;
    contextSetGlobal: () => void;
    contextGoHere: () => void;
    contextSetTakeoff: () => void;
    setContextAnchor: (e: L.LeafletMouseEvent) => void;
    contextAnchor: L.LeafletMouseEvent;
    environmentBoundary: GlobalTypes.PositionType[];
    drawPolygonPts?: GlobalTypes.PositionType[];
    onAddPolygonPt: (e: L.LeafletMouseEvent) => void;
    environmentSettings: GlobalTypes.EnvironmentSettingsType;
    gridPts?: { inPoly: L.LatLng[]; trimmedPts: L.LatLng[] };
    envBoundingBox?: GlobalTypes.PositionType[];
};

type State = {
    showContextMenu?: boolean;
    dragging?: boolean;
};

export default class MACEMap extends React.Component<Props, State> {
    leafletMap: any;
    heatmap: Heatmap;
    constructor(props: Props) {
        super(props);

        this.state = {
            showContextMenu: false,
            dragging: false
        };
    }

    componentDidMount() {}

    handleMarkerClick = (e: L.LeafletMouseEvent, vehicleId: string, type: string) => {
        this.props.handleSelectedAircraftUpdate(vehicleId);
    };

    triggerContextMenu = (event: L.LeafletMouseEvent) => {
        this.props.setContextAnchor(event);
        this.setState({ showContextMenu: !this.state.showContextMenu });
    };

    handleMapClick = (event: L.LeafletMouseEvent) => {
        if (!this.state.dragging) {
            this.props.onAddPolygonPt(event);
        }
    };

    render() {
        const globalOriginMarker = {
            position: new L.LatLng(this.props.globalOrigin.lat, this.props.globalOrigin.lng),
            icon: new L.Icon({
                iconUrl: "./images/userlocation_icon.png",
                iconSize: [41, 41], // size of the icon
                iconAnchor: [20, 20], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor,
            })
        };

        const drawingMarkers = [];
        let icon = new L.Icon({
            iconUrl: "./images/marker-icon-orange.png",
            iconSize: [25, 41], // size of the icon
            iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
            popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
        });
        for (let i = 0; i < this.props.drawPolygonPts.length; i++) {
            drawingMarkers.push(<Marker key={i} position={this.props.drawPolygonPts[i]} title={i.toString()} icon={icon} draggable={false} />);
        }

        let tmpPts = [];
        for (let i = 0; i < this.props.drawPolygonPts.length; i++) {
            tmpPts.push(this.props.drawPolygonPts[i]);
        }

        let tmpBBoxPts = [];
        if (this.props.drawPolygonPts.length > 2) {
            for (let i = 0; i < this.props.envBoundingBox.length; i++) {
                tmpBBoxPts.push(this.props.envBoundingBox[i]);
            }
        }

        let tmpGridPts = [];
        if (this.props.drawPolygonPts.length > 2) {
            let gridIcon = new L.Icon({
                iconUrl: "./images/ic_add_white_24dp_1x.png",
                iconSize: [25, 25], // size of the icon
                iconAnchor: [12, 12], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
            });
            for (let i = 0; i < this.props.gridPts.inPoly.length; i++) {
                tmpGridPts.push(<Marker key={i} position={this.props.gridPts.inPoly[i]} title={i.toString()} icon={gridIcon} draggable={false} />);
            }
        }

        let tmpTrimmedGridPts = [];
        if (this.props.drawPolygonPts.length > 2) {
            let trimmedIcon = new L.Icon({
                iconUrl: "./images/ic_add_grey600_48dp.png",
                iconSize: [25, 25], // size of the icon
                iconAnchor: [12, 12], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
            });
            for (let i = 0; i < this.props.gridPts.trimmedPts.length; i++) {
                tmpTrimmedGridPts.push(
                    <Marker key={i} position={this.props.gridPts.trimmedPts[i]} title={i.toString()} icon={trimmedIcon} draggable={false} />
                );
            }
        }

        return (
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div style={styles.parentStyle}>
                    {this.state.showContextMenu && (
                        <ContextMenu
                            menuAnchor={this.props.contextAnchor}
                            handleClose={() => this.setState({ showContextMenu: false })}
                            handleSetHome={() => {
                                this.setState({ showContextMenu: false });
                                this.props.contextSetHome();
                            }}
                            handleSetGlobal={() => {
                                this.setState({ showContextMenu: false });
                                this.props.contextSetGlobal();
                            }}
                            handleGoHere={() => {
                                this.setState({ showContextMenu: false });
                                this.props.contextGoHere();
                            }}
                            handleSetTakeoff={() => {
                                this.setState({ showContextMenu: false });
                                this.props.contextSetTakeoff();
                            }}
                            selectedVehicleID={this.props.selectedVehicleID}
                        />
                    )}

                    <Map
                        ref={(map: any) => {
                            this.leafletMap = map;
                        }}
                        ondragend={this.props.updateMapCenter}
                        useFlyTo={true}
                        animate={true}
                        center={this.props.mapCenter}
                        zoom={this.props.mapZoom}
                        style={styles.mapStyle}
                        zoomControl={false}
                        maxZoom={this.props.maxZoom}
                        oncontextmenu={this.triggerContextMenu}
                        onclick={this.handleMapClick}
                    >
                        {/* <TileLayer url='http://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}' />  */}
                        <TileLayer
                            url="http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"
                            maxZoom={this.props.maxZoom}
                            subdomains={["mt0", "mt1", "mt2", "mt3"]}
                        />

                        <LayerGroup>
                            {/* Aircraft Icons */}
                            {Object.keys(this.props.connectedVehicles).map((key: string) => {
                                return (
                                    <Marker
                                        zIndexOffset={1000}
                                        onclick={(e: L.LeafletMouseEvent) => this.handleMarkerClick(e, key, "vehicle")}
                                        key={key}
                                        position={this.props.connectedVehicles[key].vehicleMarker.latLon}
                                        icon={this.props.connectedVehicles[key].vehicleMarker.icon}
                                        title={key}
                                    />
                                );
                            })}

                            {/* Home Icons */}
                            {Object.keys(this.props.connectedVehicles).map((key: string) => {
                                return (
                                    <Marker
                                        onclick={(e: L.LeafletMouseEvent) => this.handleMarkerClick(e, key, "home")}
                                        key={key}
                                        position={this.props.connectedVehicles[key].homePosition.latLon}
                                        icon={this.props.connectedVehicles[key].homePosition.icon}
                                        title={key}
                                    />
                                );
                            })}

                            {/* Global Origin */}
                            <Marker position={globalOriginMarker.position} icon={globalOriginMarker.icon} />

                            {/* Mission Paths */}
                            {Object.keys(this.props.connectedVehicles).map((key: string) => {
                                return (
                                    <Polyline
                                        key={key}
                                        positions={this.props.connectedVehicles[key].vehicleMission.latLons}
                                        color={
                                            this.props.selectedVehicleID === key
                                                ? this.props.connectedVehicles[key].highlightColor
                                                : this.props.connectedVehicles[key].opaqueHighlightColor
                                        }
                                    />
                                );
                            })}

                            {/* Mission Markers */}
                            {Object.keys(this.props.connectedVehicles).map((key: string) => {
                                let markers: JSX.Element[] = [];
                                for (let i = 0; i < this.props.connectedVehicles[key].vehicleMission.latLons.length; i++) {
                                    markers.push(
                                        <Marker
                                            key={i}
                                            position={this.props.connectedVehicles[key].vehicleMission.latLons[i]}
                                            icon={this.props.connectedVehicles[key].vehicleMission.icons[i]}
                                            title={key}
                                        />
                                    );
                                }
                                return markers;
                            })}

                            {/* Sensor Footprint */}
                            {Object.keys(this.props.connectedVehicles).map((key: string) => {
                                return (
                                    <Polygon
                                        key={key}
                                        positions={this.props.connectedVehicles[key].sensorFootprint}
                                        color={
                                            this.props.selectedVehicleID === key
                                                ? this.props.connectedVehicles[key].highlightColor
                                                : this.props.connectedVehicles[key].opaqueHighlightColor
                                        }
                                        fillColor={colors.amber500}
                                    />
                                );
                            })}

                            {/* EnvironmentBoundary */}
                            <Polygon positions={this.props.environmentBoundary} color={colors.white} fillColor={colors.green300} />

                            {/* Guided target */}
                            {Object.keys(this.props.connectedVehicles).map((key: string) => {
                                if (this.props.connectedVehicles[key].vehicleMode === "GUIDED")
                                    return (
                                        <Marker
                                            key={key}
                                            position={this.props.connectedVehicles[key].currentTarget.targetPosition}
                                            icon={this.props.connectedVehicles[key].currentTarget.icon}
                                            title={key}
                                        />
                                    );
                            })}
                            {/* Guided target Paths */}
                            {Object.keys(this.props.connectedVehicles).map((key: string) => {
                                if (this.props.connectedVehicles[key].currentTarget.active && this.props.connectedVehicles[key].vehicleMode === "GUIDED") {
                                    return (
                                        <Polyline
                                            key={key}
                                            positions={[
                                                this.props.connectedVehicles[key].vehicleMarker.latLon,
                                                this.props.connectedVehicles[key].currentTarget.targetPosition
                                            ]}
                                            color={
                                                this.props.selectedVehicleID === key
                                                    ? this.props.connectedVehicles[key].highlightColor
                                                    : this.props.connectedVehicles[key].opaqueHighlightColor
                                            }
                                        />
                                    );
                                }
                            })}

                            {/* Bounding box */}
                            {this.props.environmentSettings.showBoundingBox && (
                                <Polygon positions={tmpBBoxPts} color={colors.blue100} fillColor={colors.blue100} />
                            )}
                            {/* Grid points */}
                            {tmpGridPts}
                            {/* Trimmed points */}
                            {this.props.environmentSettings.showBoundingBox && tmpTrimmedGridPts}

                            {/* Drawing polygon */}
                            <Polygon positions={tmpPts} color={colors.white} fillColor={colors.purple400} />
                            {drawingMarkers}
                        </LayerGroup>
                    </Map>
                </div>
            </MuiThemeProvider>
        );
    }
}
