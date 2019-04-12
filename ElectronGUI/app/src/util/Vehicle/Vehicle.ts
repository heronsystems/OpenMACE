import * as L from "leaflet";
import * as GlobalTypes from "../../types/globalTypings";
import { aircraftImgSrcFromType } from "../Helpers/VehicleHelper";

export class Vehicle {
    isNew: boolean;
    general: GlobalTypes.HeartbeatType & { lastHeard: Date };
    vehicleId: number;
    isSelected: boolean;
    position: GlobalTypes.PositionType;
    attitude: GlobalTypes.AttitudeType;
    fuel: GlobalTypes.FuelType;
    numSats: number;
    positionFix: number;
    vehicleMode: GlobalTypes.VehicleModeType;
    positionInterval: number;
    attitudeInterval: number;
    vehicleMarker: GlobalTypes.MarkerType;
    vehicleMission: GlobalTypes.MissionLayerType;
    homePosition: GlobalTypes.MarkerType;
    isArmed: boolean;
    sensorFootprint: L.LatLng[];
    currentMissionItem: number;
    messages: GlobalTypes.TextType[];
    airspeed: number;
    gps: GlobalTypes.GPSType;
    highlightColor: string;
    opaqueHighlightColor: string;
    currentTarget: {
        distanceToTarget: number;
        targetPosition: GlobalTypes.PositionType;
        icon: L.DivIcon;
        active: boolean;
    };
    availableModes: string[];

    constructor(vehicleId: number, position?: GlobalTypes.PositionType, attitude?: GlobalTypes.AttitudeType) {
        this.isNew = true;
        this.vehicleId = vehicleId;
        this.isSelected = false;
        this.isArmed = false;
        this.numSats = 0;
        this.positionFix = 0;
        this.vehicleMode = "UNKNOWN";
        this.fuel = {
            batteryCurrent: 0,
            batteryRemaining: 0,
            batteryVoltage: 0
        };
        this.sensorFootprint = [];
        this.currentMissionItem = 0;
        this.messages = [];
        this.airspeed = 0;
        this.general = {
            commsProtocol: "COMMS_UNKNOWN",
            autopilot: "AUTOPILOT_TYPE_GENERIC",
            aircraftType: "GENERIC",
            companion: false,
            lastHeard: new Date()
        };
        this.gps = {
            visibleSats: 0,
            gpsFix: "No GPS",
            hdop: 20, // > 20 is poor level of confidence
            vdop: 20 // > 20 is poor level of confidence
        };
        if (position) {
            this.position = position;
        } else {
            this.position = { lat: 0, lng: 0, alt: 0 };
            // this.position = {lat: -35.363272, lon: 149.165249, alt: 0};
        }
        if (attitude) {
            this.attitude = attitude;
        } else {
            this.attitude = { roll: 0, pitch: 0, yaw: 0 };
        }

        let vehicleIconHTML =
            "<div style='background-color: " +
            this.opaqueHighlightColor +
            "; color: white; width: 41px; text-align: center;'>" +
            this.vehicleId +
            "</div><img src='./images/drone-icon.png' alt='Drone icon' style='width:41px; height:41px; -webkit-transform: rotate(" +
            this.attitude.yaw +
            "deg); -moz-transform: rotate(" +
            this.attitude.yaw +
            "deg); -o-transform: rotate(" +
            this.attitude.yaw +
            "deg); -ms-transform: rotate(" +
            this.attitude.yaw +
            "deg); transform: rotate(" +
            this.attitude.yaw +
            "deg);'>";

        this.vehicleMarker = {
            vehicleId: this.vehicleId,
            latLon: new L.LatLng(this.position.lat, this.position.lng),
            altitude: this.position.alt,
            icon: new L.DivIcon({
                html: vehicleIconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18], // point from which the popup should open relative to the iconAnchor
                className: "" // setting this overrides default css and gets rid of white box
            })
        };

        // Set blank mission
        this.vehicleMission = {
            descriptions: [],
            latLons: [],
            itemTypes: [],
            icons: [],
            missionID: 0,
            creatorID: 0,
            missionState: "UNEXECUTED",
            missionType: "AUTO"
        };

        // Set default homePosition
        let homeIcon = new L.Icon({
            iconUrl: "./images/Home-icon.png",
            iconSize: [25, 41], // size of the icon
            iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
            popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
        });
        this.homePosition = {
            vehicleId: this.vehicleId,
            // latLon: new L.LatLng(this.position.lat, this.position.lon),
            latLon: new L.LatLng(0, 0),
            altitude: this.position.alt,
            icon: homeIcon
        };

        let currentTargetHTML = "<img src='./images/guided-icon.png' alt='Guided icon' style='width:41px; height:41px;'>";
        this.currentTarget = {
            distanceToTarget: null,
            targetPosition: { lat: null, lng: null, alt: null },
            icon: new L.DivIcon({
                html: currentTargetHTML,
                iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
                popupAnchor: [-19, 38], // point from which the popup should open relative to the iconAnchor
                className: "" // setting this overrides default css and gets rid of white box
            }),
            active: false
        };

        this.availableModes = [];
    }

    setAvailableVehicleModes() {
        if (this.general.aircraftType === "GENERIC") {
        } else if (this.general.aircraftType === "HELICOPTER") {
            this.availableModes = [
                "STABILIZE",
                "ACRO",
                "ALT_HOLD",
                "AUTO",
                "GUIDED",
                "LOITER",
                "RTL",
                "CIRCLE",
                "LAND",
                "DRIFT",
                "SPORT",
                "FLIP",
                "AUTOTUNE",
                "POSHOLD",
                "BRAKE",
                "THROW",
                "AVOID_ADSB",
                "GUIDED_NOGPS",
                "UNKNOWN"
            ];
        } else if (this.general.aircraftType === "GCS") {
        } else if (this.general.aircraftType === "REPEATER") {
        } else if (this.general.aircraftType === "GROUND_ROVER") {
        } else if (this.general.aircraftType === "SURFACE_BOAT") {
        } else if (this.general.aircraftType === "TRICOPTER") {
            this.availableModes = [
                "STABILIZE",
                "ACRO",
                "ALT_HOLD",
                "AUTO",
                "GUIDED",
                "LOITER",
                "RTL",
                "CIRCLE",
                "LAND",
                "DRIFT",
                "SPORT",
                "FLIP",
                "AUTOTUNE",
                "POSHOLD",
                "BRAKE",
                "THROW",
                "AVOID_ADSB",
                "GUIDED_NOGPS",
                "UNKNOWN"
            ];
        } else if (this.general.aircraftType === "QUADROTOR") {
            this.availableModes = [
                "STABILIZE",
                "ACRO",
                "ALT_HOLD",
                "AUTO",
                "GUIDED",
                "LOITER",
                "RTL",
                "CIRCLE",
                "LAND",
                "DRIFT",
                "SPORT",
                "FLIP",
                "AUTOTUNE",
                "POSHOLD",
                "BRAKE",
                "THROW",
                "AVOID_ADSB",
                "GUIDED_NOGPS",
                "UNKNOWN"
            ];
        } else if (this.general.aircraftType === "HEXAROTOR") {
            this.availableModes = [
                "STABILIZE",
                "ACRO",
                "ALT_HOLD",
                "AUTO",
                "GUIDED",
                "LOITER",
                "RTL",
                "CIRCLE",
                "LAND",
                "DRIFT",
                "SPORT",
                "FLIP",
                "AUTOTUNE",
                "POSHOLD",
                "BRAKE",
                "THROW",
                "AVOID_ADSB",
                "GUIDED_NOGPS",
                "UNKNOWN"
            ];
        } else if (this.general.aircraftType === "OCTOROTOR") {
            this.availableModes = [
                "STABILIZE",
                "ACRO",
                "ALT_HOLD",
                "AUTO",
                "GUIDED",
                "LOITER",
                "RTL",
                "CIRCLE",
                "LAND",
                "DRIFT",
                "SPORT",
                "FLIP",
                "AUTOTUNE",
                "POSHOLD",
                "BRAKE",
                "THROW",
                "AVOID_ADSB",
                "GUIDED_NOGPS",
                "UNKNOWN"
            ];
        } else if (this.general.aircraftType === "ONBOARD_CONTROLLER") {
        } else if (this.general.aircraftType === "FIXED_WING") {
            this.availableModes = [
                "MANUAL",
                "CIRCLE",
                "STABILIZE",
                "TRAINING",
                "ACRO",
                "FLY_BY_WIRE_A",
                "FLY_BY_WIRE_B",
                "CRUISE",
                "AUTOTUNE",
                "AUTO",
                "RTL",
                "LOITER",
                "AVOID_ADSB",
                "GUIDED",
                "INITIALISING",
                "QSTABILIZE",
                "QHOVER",
                "QLOITER",
                "QLAND",
                "QRTL",
                "UNKNOWN",
                "NR"
            ];
        }
    }

    setPosition(position: GlobalTypes.PositionType) {
        this.position = position;
    }

    setAttitude(attitude: GlobalTypes.AttitudeType) {
        this.attitude = attitude;
    }

    setVehicleMode(vehicleMode: GlobalTypes.VehicleModeType) {
        this.vehicleMode = vehicleMode;
    }

    setNumSats(numSats: number) {
        this.numSats = numSats;
    }

    setPositionFix(positionFix: number) {
        this.positionFix = positionFix;
    }

    setVehicleMission(mission: GlobalTypes.TCPMissionType) {
        let prevLatLng = this.homePosition.latLon;
        // Clear old mission:
        this.vehicleMission = {
            descriptions: [],
            latLons: [],
            itemTypes: [],
            icons: []
        };
        for (let i = 0; i < mission.missionItems.length; i++) {
            this.vehicleMission.descriptions.push(mission.missionItems[i].description);
            this.vehicleMission.itemTypes.push(mission.missionItems[i].type);
            let tmpLatLng = new L.LatLng(
                mission.missionItems[i].lat ? mission.missionItems[i].lat : prevLatLng.lat,
                mission.missionItems[i].lng ? mission.missionItems[i].lng : prevLatLng.lng
            );
            this.vehicleMission.latLons.push(tmpLatLng);

            let tmpIcon = this.getMarkerIcon(mission.missionItems[i].type, false);
            this.vehicleMission.icons.push(tmpIcon as L.Icon);

            // Set prevLatLng to this lat lng
            prevLatLng = tmpLatLng;
        }

        // If set, set the meta data:
        if (mission.creatorID) {
            this.vehicleMission.creatorID = mission.creatorID;
        }
        if (mission.missionID) {
            this.vehicleMission.missionID = mission.missionID;
        }
        if (mission.missionState) {
            this.vehicleMission.missionState = mission.missionState;
        }
        if (mission.missionType) {
            this.vehicleMission.missionType = mission.missionType;
        }
    }

    getMarkerIcon(type: string, isActive: boolean) {
        let iconBackgroundColor = this.opaqueHighlightColor;
        let iconUrl = "./images/marker-icon.png";
        let width = 25;
        let height = 41;
        let iconAnchor: [number, number] = [12, 41];
        let popupAnchor: [number, number] = [0, -38];

        if (type === "WAYPOINT") {
            iconUrl = "./images/marker-icon.png";
            width = 25;
            height = 41;
            iconAnchor = [12, 41];
            popupAnchor = [0, -38];
        } else if (type === "LOITER_UNLIMITED") {
            // TODO: Figure out a way to display radius
            iconUrl = "./images/Circle-icon.png";
            width = 41;
            height = 41;
            iconAnchor = [20, 20];
            popupAnchor = [0, -38];
        } else if (type === "LOITER_TURNS") {
            // TODO: Figure out a way to display radius
            iconUrl = "./images/Circle-icon.png";
            width = 41;
            height = 41;
            iconAnchor = [20, 20];
            popupAnchor = [0, -38];
        } else if (type === "LOITER_TIME") {
            // TODO: Figure out a way to display radius
            iconUrl = "./images/Circle-icon.png";
            width = 41;
            height = 41;
            iconAnchor = [20, 20];
            popupAnchor = [0, -38];
        } else if (type === "ARM") {
            iconUrl = "./images/marker-icon-orange.png";
            width = 25;
            height = 41;
            iconAnchor = [12, 41];
            popupAnchor = [0, -38];
        } else if (type === "CHANGE_MODE") {
            // TODO
        } else if (type === "CHANGE_SPEED") {
            // TODO
        } else if (type === "MOTOR_TEST") {
            // TODO
        } else if (type === "TAKEOFF") {
            iconUrl = "./images/ic_wp_takeof_selected.png";
            width = 41;
            height = 41;
            iconAnchor = [20, 20];
            popupAnchor = [0, -38];
        } else if (type === "LAND") {
            iconUrl = "./images/ic_wp_land_selected.png";
            width = 41;
            height = 41;
            iconAnchor = [20, 20];
            popupAnchor = [0, -38];
        } else if (type === "RTL") {
            iconUrl = "./images/Home-Icon-Selected.png";
            width = 31;
            height = 31;
            iconAnchor = [15, 15];
            popupAnchor = [0, -38];
        }

        if (isActive) {
            let iconHTML = "<div style='width: " + width + "; height: " + (height + 18) + "'>";
            iconHTML += "<img src='" + iconUrl + "'alt='Marker icon' style='width: " + width + "px; height: " + height + "px;'>";
            iconHTML +=
                "<span style='background-color: " +
                iconBackgroundColor +
                "; color: white; text-align: center; padding: 2px; position: relative; top: " +
                (-height - 18) +
                "px; left: -5px'>active</span>";
            iconHTML += "</div>";

            let icon = new L.DivIcon({
                html: iconHTML,
                iconAnchor: iconAnchor, // point of the icon which will correspond to marker's location
                popupAnchor: popupAnchor, // point from which the popup should open relative to the iconAnchor
                className: "" // setting this overrides default css and gets rid of white box
            });

            // Return icon:
            return icon;
        } else {
            let icon = new L.Icon({
                iconUrl: iconUrl,
                iconSize: [width, height], // size of the icon
                iconAnchor: iconAnchor, // point of the icon which will correspond to marker's location
                popupAnchor: popupAnchor // point from which the popup should open relative to the iconAnchor
            });
            // Return icon:
            return icon;
        }
    }

    updateHomePosition(newHome?: { lat: number; lon: number; alt: number }) {
        let homeUpdate = {
            lat: this.homePosition.latLon.lat,
            lon: this.homePosition.latLon.lng,
            alt: this.homePosition.altitude
        };
        if (newHome) {
            homeUpdate = newHome;
        }
        let iconBackgroundColor = this.isSelected ? this.highlightColor : this.opaqueHighlightColor;
        let iconHTML =
            "<div style='background-color: " +
            iconBackgroundColor +
            "; color: white; width: 41px; text-align: center;'>" +
            this.vehicleId +
            "</div><img src='./images/Home-Icon.png' alt='Home icon' style='width:41px; height:41px; '>";

        this.homePosition = {
            vehicleId: this.vehicleId,
            latLon: new L.LatLng(homeUpdate.lat, homeUpdate.lon),
            altitude: 0,
            icon: new L.DivIcon({
                html: iconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18], // point from which the popup should open relative to the iconAnchor
                className: "" // setting this overrides default css and gets rid of white box
            })
        };
    }

    updateVehicleMarkerPosition(newPos?: GlobalTypes.PositionType) {
        let posUpdate = this.position;
        if (newPos) {
            posUpdate = newPos;
        }

        let iconBackgroundColor = this.isSelected ? this.highlightColor : this.opaqueHighlightColor;
        let aircraftIconSrc = aircraftImgSrcFromType(this.general.aircraftType);
        let iconHTML =
            "<div style='background-color: " +
            iconBackgroundColor +
            "; color: white; width: 41px; text-align: center;'>" +
            this.vehicleId +
            "</div><img src='" +
            aircraftIconSrc +
            "' alt='Drone icon' style='width:41px; height:41px; -webkit-transform: rotate(" +
            this.attitude.yaw +
            "deg); -moz-transform: rotate(" +
            this.attitude.yaw +
            "deg); -o-transform: rotate(" +
            this.attitude.yaw +
            "deg); -ms-transform: rotate(" +
            this.attitude.yaw +
            "deg); transform: rotate(" +
            this.attitude.yaw +
            "deg);'>";

        this.vehicleMarker = {
            vehicleId: this.vehicleId,
            latLon: new L.LatLng(posUpdate.lat, posUpdate.lng),
            altitude: 0,
            icon: new L.DivIcon({
                html: iconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18], // point from which the popup should open relative to the iconAnchor
                className: "" // setting this overrides default css and gets rid of white box
            })
        };
    }

    updateMarkerAttitude(newAtt?: GlobalTypes.AttitudeType) {
        let attUpdate = this.attitude;
        if (newAtt) {
            attUpdate = newAtt;
        }

        let iconBackgroundColor = this.isSelected ? this.highlightColor : this.opaqueHighlightColor;
        let aircraftIconSrc = aircraftImgSrcFromType(this.general.aircraftType);
        let iconHTML =
            "<div style='background-color: " +
            iconBackgroundColor +
            "; color: white; width: 41px; text-align: center;'>" +
            this.vehicleId +
            "</div><img src='" +
            aircraftIconSrc +
            "' alt='Drone icon' style='width:41px; height:41px; -webkit-transform: rotate(" +
            attUpdate.yaw +
            "deg); -moz-transform: rotate(" +
            attUpdate.yaw +
            "deg); -o-transform: rotate(" +
            attUpdate.yaw +
            "deg); -ms-transform: rotate(" +
            attUpdate.yaw +
            "deg); transform: rotate(" +
            attUpdate.yaw +
            "deg);'>";

        this.vehicleMarker = {
            vehicleId: this.vehicleId,
            latLon: new L.LatLng(this.position.lat, this.position.lng),
            altitude: 0,
            icon: new L.DivIcon({
                html: iconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18], // point from which the popup should open relative to the iconAnchor
                className: "" // setting this overrides default css and gets rid of white box
            })
        };
    }

    updateSensorFootprint(newFootprint: GlobalTypes.PositionType[]) {
        this.sensorFootprint = [];
        for (let i = 0; i < newFootprint.length; i++) {
            let vertex = new L.LatLng(newFootprint[i].lat, newFootprint[i].lng);
            this.sensorFootprint.push(vertex);
        }
    }

    updateCurrentMissionItem(currentMissionItem: number, clearActive: boolean) {
        let previousItem = this.currentMissionItem;
        // Set previous mission item icon back to original:
        let prevIcon = this.getMarkerIcon(this.vehicleMission.itemTypes[previousItem], false);
        this.vehicleMission.icons[previousItem] = prevIcon as L.Icon;

        // Set new mission item icon and current mission item. If clearActive flag is set, don't show active on top of the icon:
        let currentIcon = new L.DivIcon({});
        if (clearActive) {
            currentIcon = this.getMarkerIcon(this.vehicleMission.itemTypes[currentMissionItem], false);
        } else {
            currentIcon = this.getMarkerIcon(this.vehicleMission.itemTypes[currentMissionItem], true);
        }
        this.vehicleMission.icons[currentMissionItem] = currentIcon as L.Icon;
        this.currentMissionItem = currentMissionItem;
    }
}
