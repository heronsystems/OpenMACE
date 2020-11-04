
export type Notification = {
    title: string;
    message: string;
    type: "danger" | "success" | "info" | "warning" | "default";
}

export type Vertex = { lat: number; lng: number; alt?: number };
type Vertices = Vertex[];
export type ColorObject = {
    100: string;
    200: string;
    300: string;
    400: string;
    500: string;
    600: string;
    700: string;
    800: string;
    900: string;
};

export type MessageType =
    | EnvironmentBoundary_MessageType
    | EnvironmentIcon_MessageType
    | VehicleHeartbeat_MessageType
    | VehiclePosition_MessageType
    | VehicleAttitude_MessageType
    | VehicleAirspeed_MessageType
    | VehicleArm_MessageType
    | VehicleGPS_MessageType
    | VehicleText_MessageType
    | VehicleMode_MessageType
    | VehicleFuel_MessageType
    | VehicleTarget_MessageType
    | VehiclePath_MessageType
    | VehicleParameterList_MessageType

interface IMessage {
    message_type: MessageType;
    date?: number;
}

type EnvironmentBoundary_MessageType = "environment_boundary";
type EnvironmentIcon_MessageType = "environment_icon";
type VehicleHeartbeat_MessageType = "vehicle_heartbeat";
type VehiclePosition_MessageType = "vehicle_position";
type VehicleAttitude_MessageType = "vehicle_attitude";
type VehicleAirspeed_MessageType = "vehicle_airspeed";
type VehicleArm_MessageType = "vehicle_arm";
type VehicleGPS_MessageType = "vehicle_gps";
type VehicleText_MessageType = "vehicle_text";
type VehicleMode_MessageType = "vehicle_mode";
type VehicleFuel_MessageType = "vehicle_fuel";
type VehicleTarget_MessageType = "vehicle_target";
type VehiclePath_MessageType = "vehicle_path";
type VehicleParameterList_MessageType = "vehicle_parameter_list"

export type Message =
    | Environment.Boundary
    | Environment.Icon
    | Aircraft.Heartbeat
    | Aircraft.Position
    | Aircraft.Attitude
    | Aircraft.Airspeed
    | Aircraft.Arm
    | Aircraft.GPS
    | Aircraft.Text
    | Aircraft.Mode
    | Aircraft.Fuel
    | Aircraft.Path
    | Aircraft.Target
    | Aircraft.Parameters

export namespace Environment {
    export type BoundaryType = "soft" | "hard";
    export type IconType =
        | "command_control"
        | "takeoff_land"
        | "origin"
        | "mission_target";
    export type BoundaryPayload = {
        boundary_name: string;
        boundary_type: BoundaryType;
        vertices: Vertices;
        should_display?: boolean;
    };

    export type IconPayload = {
        agentID: string;
        name: string;
        type: IconType;
        lat: number;
        lng: number;
        alt?: number;
        should_display?: boolean;
        auto_focus?: boolean;
        lastUpdate?: number;
    };

    export interface Boundary extends IMessage {
        message_type: EnvironmentBoundary_MessageType;
        boundary_name: string;
        boundary_type: BoundaryType;
        vertices: Vertices;
        should_display?: boolean;
    }

    export interface Icon extends IMessage {
        message_type: EnvironmentIcon_MessageType;
        name: string;
        type: IconType;
        lat: number;
        lng: number;
        alt?: number;
        should_display?: boolean;
        auto_focus?: boolean;
        lastUpdate?: number;
    }
}

export namespace Aircraft {
    export type AircraftPayload = {
        agentID: string;
        selected: boolean;
        should_display?: boolean;
        color?: ColorObject;
        date?: number;
        lastUpdate?: number;
        vehicle_type: string;
        behavior_state: string;
        vehicle_state: string;
        orientation: {
            pitch: number;
            roll: number;
            yaw: number;
        };
        location: Vertex;
        armed: boolean;
        visible_sats: number;
        gps_fix: string;
        hdop: number;
        vdop: number;
        text: {
            textStr: string;
            textSeverity: string;
            textTimestamp: number;
        };
        mode: string;
        battery_remaining: number;
        battery_current: number;
        battery_voltage: number;
        param_list: {
            param_id: string;
            value: number
        }[];
        airspeed: number;
        distance_to_target: number;
        flight_time: number;
    }

    export type HeartbeatPayload = {
        agentID: string;
        autopilot: string;
        vehicle_type: string;
        companion: boolean;
        protocol: string;
        mission_state: number;
        mavlink_id: number;
        behavior_state: string;
        vehicle_state: string;
        date?: number;
        should_display?: boolean;
        lastUpdate?: number;
    };
    export interface Heartbeat extends IMessage {
        message_type: VehicleHeartbeat_MessageType;
        agentID: string;
        autopilot: string;
        vehicle_type: string;
        companion: boolean;
        protocol: string;
        mission_state: number;
        mavlink_id: number;
        behavior_state: string;
        vehicle_state: string;
        date?: number;
        should_display?: boolean;
        lastUpdate?: number;
    }

    export type PositionPayload = {
        agentID: string;
        should_display?: boolean;
        lat: number;
        lng: number;
        alt: number;
    }

    export interface Position {
        message_type: VehiclePosition_MessageType;
        agentID: string;
        should_display?: boolean;
        lat: number;
        lng: number;
        alt: number;
    }

    export type AttitudePayload = {
        agentID: string;
        should_display?: boolean;
        roll: number;
        pitch: number;
        yaw: number;
    }

    export interface Attitude {
        message_type: VehicleAttitude_MessageType;
        agentID: string;
        should_display?: boolean;
        roll: number;
        pitch: number;
        yaw: number;
    }

    export type AirspeedPayload = {
        agentID: string;
        should_display?: boolean;
        airspeed: number;
    }

    export interface Airspeed {
        message_type: VehicleAirspeed_MessageType;
        agentID: string;
        should_display?: boolean;
        airspeed: number;
    }


    export type ArmPayload = {
        agentID: string;
        should_display?: boolean;
        armed: boolean;
    }

    export interface Arm {
        message_type: VehicleArm_MessageType;
        agentID: string;
        should_display?: boolean;
        armed: boolean;
    }

    export type GPSPayload = {
        agentID: string;
        should_display?: boolean;
        visible_sats: number;
        gps_fix: string;
        hdop: number;
        vdop: number;
    }

    export interface GPS {
        message_type: VehicleGPS_MessageType;
        agentID: string;
        should_display?: boolean;
        visible_sats: number;
        gps_fix: string;
        hdop: number;
        vdop: number;
    }

    export type TextPayload = {
        agentID: string;
        should_display?: boolean;
        text: string;
        severity: string;
    }

    export interface Text {
        message_type: VehicleText_MessageType;
        agentID: string;
        should_display?: boolean;
        text: string;
        severity: string;
    }

    export type ModePayload = {
        agentID: string;
        should_display?: boolean;
        mode: string;
    }

    export interface Mode {
        message_type: VehicleMode_MessageType;
        agentID: string;
        should_display?: boolean;
        mode: string;
    }

    export type FuelPayload = {
        agentID: string;
        should_display?: boolean;
        battery_remaining: number;
        battery_current: number;
        battery_voltage: number;
    }

    export interface Fuel {
        message_type: VehicleFuel_MessageType;
        agentID: string;
        should_display?: boolean;
        battery_remaining: number;
        battery_current: number;
        battery_voltage: number;
    }


    export type PathPayload = {
        agentID: string;
        vertices: Vertices;
        date?: number;
        should_display?: boolean;
        lastUpdate?: number;
    };

    export interface Path {
        message_type: VehiclePath_MessageType;
        agentID: string;
        vertices: Vertices;
        date?: number;
        should_display?: boolean;
        lastUpdate?: number;
    }

    export type TargetPayload = {
        agentID?: string;
        location: Vertex;
        date?: number;
        should_display?: boolean;
        is_global?: boolean;
        distance_to_target: number;
        lastUpdate?: number;
    };

    export interface Target {
        message_type: VehicleTarget_MessageType;
        agentID: string;
        location: Vertex;
        date?: number;
        should_display?: boolean;
        is_global?: boolean;
        lastUpdate?: number;
    }

    export type ParametersPayload = {
        agentID?: string;
        param_list: {
            param_id: string;
            value: number;
        }[];
        should_display?: boolean;
    }

    export interface Parameters {
        message_type: VehicleParameterList_MessageType;
        agentID?: string;
        param_list: {
            param_id: string;
            value: number;
        }[];
        should_display?: boolean;
    }
}

