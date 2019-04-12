import * as L from "leaflet";

/*
TODO: Figure out how to define this so we can pass it to AppHelper as a type STATE:

  MACEconfig?: ConfigSettingsType,
  connectedVehicles?: {[id: string]: Vehicle}
  vehicleWarnings?: VehicleWarning[]
  selectedVehicleID?: string,
  openDrawer?: boolean,
  tcpServer?: any,
  allowVehicleSelect?: boolean,
  showEditVehicleHomeDialog?: boolean,
  showEditGlobalHomeDialog?: boolean,
  showMessagesMenu?: boolean,
  showConfigDialog?: boolean,
  messagePreferences?: MessagePreferencesType,
  showTakeoffDialog?: boolean,
  showSaveTakeoff?: boolean,
  globalOrigin?: PositionType
  useContext?: boolean,
  contextAnchor?: L.LeafletMouseEvent,
  MACEConnected?: boolean,
  environmentBoundary?: PositionType[],
  showDraw?: boolean,
  drawPolygonPts?: PositionType[],
  gridPts?: {inPoly: L.LatLng[], trimmedPts: L.LatLng[]},
  showEnvironmentSettings?: boolean,
  environmentSettings?: EnvironmentSettingsType,
  pauseMACEComms?: boolean,
  envBoundingBox?: PositionType[],
  getConnectedVehiclesTimeout?: number


*/

export type MACEConfig = {
    MACEComms?: {
        ipAddress?: string;
        listenPortNumber?: number;
        sendPortNumber?: number;
    };
    GUIInit?: {
        mapCenter?: PositionType;
        mapZoom?: number;
        maxZoom?: number;
    };
    VehicleSettings?: {
        defaultTakeoffAlt?: number;
    };
};

export type PositionType = {
    lat: number;
    lng: number;
    alt: number;
};

export type AttitudeType = {
    roll: number;
    pitch: number;
    yaw: number;
};

export type FuelType = {
    batteryRemaining: number;
    batteryCurrent: number;
    batteryVoltage: number;
};

export type TextType = {
    severity: string;
    text: string;
    timestamp: Date;
};

export type GPSType = {
    visibleSats: number;
    gpsFix: string;
    hdop: number;
    vdop: number;
};

export type VehicleModeType = "LOITER" | "RTL" | "LAND" | "AUTO" | "GUIDED" | "UNKNOWN";

export type VehicleTypeType =
    | "GENERIC"
    | "HELICOPTER"
    | "GCS"
    | "REPEATER"
    | "GROUND_ROVER"
    | "SURFACE_BOAT"
    | "TRICOPTER"
    | "QUADROTOR"
    | "HEXAROTOR"
    | "OCTOROTOR"
    | "ONBOARD_CONTROLLER"
    | "FIXED_WING";

export type HeartbeatType = {
    autopilot: string;
    aircraftType: VehicleTypeType;
    companion: boolean;
    commsProtocol: string;
};

export type TCPDescriptorType = {
    dataType: string;
    vehicleID: number;
};

export type ConnectedVehicles = {
    ids: number[];
    modes: any[];
}

export type ConnectedVehiclesType = {
    connectedVehicles: ConnectedVehicles;
};

export type TCPPositionType = PositionType & {
    positionFix: number;
    numSats: number;
};

export type TCPAttitudeType = AttitudeType;

export type TCPFuelType = FuelType;

export type TCPModeType = {
    vehicleMode: string;
};

export type TCPTextType = TextType;

export type TCPGPSType = GPSType;

export type MissionItemType = PositionType & {
    description: string;
    type: string;
};

export type TCPMissionType = {
    missionItems: MissionItemType[];
    missionType?: string;
    creatorID?: number;
    missionID?: number;
    missionState?: string;
};

export type TCPSensorFootprintType = {
    sensorFootprint: PositionType[];
};

export type TCPEnvironmentBoundaryType = {
    environmentBoundary: PositionType[];
};

export type TCPCurrentMissionItemType = {
    missionItemIndex: number;
};

export type TCPVehicleTargetType = PositionType & {
    distanceToTarget: number;
};

export type TCPVehicleArmType = {
    armed: boolean;
};

export type TCPMissionItemReachedType = {
    itemIndex: number;
};

export type TCPAirspeedType = {
    airspeed: number;
};

export type TCPHeartbeatType = HeartbeatType;

export type TCPOriginType = PositionType & {
    gridSpacing: number;
};

export type ReturnType =
    | ConnectedVehiclesType
    | TCPPositionType
    | TCPAttitudeType
    | TCPFuelType
    | TCPMissionType
    | TCPModeType
    | TCPTextType
    | TCPSensorFootprintType
    | TCPCurrentMissionItemType
    | TCPGPSType
    | TCPHeartbeatType
    | TCPMissionItemReachedType
    | TCPVehicleArmType
    | TCPAirspeedType
    | TCPEnvironmentBoundaryType
    | TCPVehicleTargetType
    | TCPOriginType;

export type TCPReturnType = TCPDescriptorType & ReturnType;

export type MarkerType = {
    latLon: L.LatLng;
    icon: L.DivIcon;
    altitude: number;
    vehicleId?: number;
};

export type LayerGroupType = {
    type: string;
    latLons: L.LatLng[];
};

export type MissionLayerType = {
    descriptions: string[];
    latLons: L.LatLng[];
    itemTypes: string[];
    icons: L.Icon[];
    missionType?: string;
    creatorID?: number;
    missionID?: number;
    missionState?: string;
};

export type MessagePreferencesType = {
    emergency: boolean;
    alert: boolean;
    critical: boolean;
    error: boolean;
    warning: boolean;
    notice: boolean;
    info: boolean;
    debug: boolean;
};

export type EnvironmentSettingsType = {
    minSliderVal: number;
    maxSliderVal: number;
    showBoundingBox: boolean;
    gridSpacing: number;
};

export type ConfigSettingsType = {
    filename?: string;
    config?: MACEConfig;
};

export type HeatmapOptions = {
    size?: number;
    units?: "m" | "px";
    opacity?: number;
    gradientTexture?: string;
    alphaRange?: number;
};
