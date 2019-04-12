import * as deepcopy from "deepcopy";
import * as GlobalTypes from "../../types/globalTypings";
import { getRandomRGB } from "../misc/Colors";
import { Vehicle } from "../Vehicle/Vehicle";

export class VehicleDB {
    vehicles: { [id: string]: Vehicle };
    globalOrigin: GlobalTypes.PositionType;
    environmentBoundary?: GlobalTypes.PositionType[];
    messagePreferences?: GlobalTypes.MessagePreferencesType;

    constructor() {
        this.vehicles = {};
        this.globalOrigin = { lat: 0, lng: 0, alt: 0 };
        this.messagePreferences = {
            emergency: true,
            alert: true,
            critical: true,
            error: true,
            warning: true,
            notice: true,
            info: true,
            debug: true
        };
        this.environmentBoundary = [];
    }

    parseJSONData = (jsonData: GlobalTypes.TCPReturnType) => {
        let stateCopy = deepcopy(this.vehicles);
        // Log message:
        // this.logger.info("[MACE Data: " + JSON.stringify(jsonData) + "]");
        let dataType: string = jsonData.dataType;
        let vehicleID: number = jsonData.vehicleID;
        if (dataType === "ConnectedVehicles") {
            let jsonVehicles = jsonData as GlobalTypes.ConnectedVehiclesType;

            console.log("Connected vehicles return: " + jsonVehicles.connectedVehicles.ids);

            // Check if vehicle is already in the map. If so, update mode. If not, add it:
            for (let i = 0; i < jsonVehicles.connectedVehicles.ids.length; i++) {
                if (stateCopy[jsonVehicles.connectedVehicles.ids[i].toString()] !== undefined) {
                    // console.log("Vehicle found: " + jsonVehicles.connectedVehicles[i]);
                    stateCopy[jsonVehicles.connectedVehicles.ids[i].toString()].vehicleMode = jsonVehicles.connectedVehicles.modes[i];
                    continue;
                } else {
                    console.log("Vehicle NOT found: " + jsonVehicles.connectedVehicles.ids[i]);
                    let newVehicle = new Vehicle(jsonVehicles.connectedVehicles.ids[i]);
                    let rgb = getRandomRGB();

                    newVehicle.highlightColor = "rgba(" + rgb.r + "," + rgb.g + "," + rgb.b + ",1)";
                    newVehicle.opaqueHighlightColor = "rgba(" + rgb.r + "," + rgb.g + "," + rgb.b + ",.2)";
                    newVehicle.vehicleMode = jsonVehicles.connectedVehicles.modes[i];
                    stateCopy[jsonVehicles.connectedVehicles.ids[i].toString()] = newVehicle;
                }
            }

            // Check if we need to remove a vehicle from the state. If we find it, continue. Else, delete it:
            let idArrays: string[] = Object.keys(stateCopy);
            for (let i = 0; i < idArrays.length; i++) {
                if (jsonVehicles.connectedVehicles.ids.indexOf(parseInt(idArrays[i])) >= 0) {
                    continue;
                } else {
                    console.log("Delete vehicle: " + idArrays[i]);
                    delete stateCopy[idArrays[i]];
                }
            }

            this.vehicles = stateCopy;
        } else if (dataType === "GlobalOrigin") {
            let jsonOrigin = jsonData as GlobalTypes.TCPOriginType;
            let origin = { lat: jsonOrigin.lat, lng: jsonOrigin.lng, alt: jsonOrigin.alt };
            this.globalOrigin = origin;

            /* TODO-PAT: Update PolygonHelper class with this data: */
            // let settings = deepcopy(this.environmentSettings);
            // settings.gridSpacing = jsonOrigin.gridSpacing;
            // this.environmentSettings = settings;
        } else if (dataType === "EnvironmentBoundary") {
            let jsonBoundary = jsonData as GlobalTypes.TCPEnvironmentBoundaryType;
            this.environmentBoundary = jsonBoundary.environmentBoundary;
        }
        // Vehicle specific data:
        else {
            // Only process if we have the vehicle in the map:
            if (stateCopy[jsonData.vehicleID]) {
                if (dataType === "VehiclePosition") {
                    let vehiclePosition = jsonData as GlobalTypes.TCPPositionType;

                    stateCopy[vehicleID].position.lat = vehiclePosition.lat;
                    stateCopy[vehicleID].position.lng = vehiclePosition.lng;
                    stateCopy[vehicleID].position.alt = vehiclePosition.alt;
                    stateCopy[vehicleID].numSats = vehiclePosition.numSats;
                    stateCopy[vehicleID].positionFix = vehiclePosition.positionFix;

                    stateCopy[vehicleID].updateVehicleMarkerPosition(vehiclePosition);

                    if (
                        stateCopy[vehicleID].isNew &&
                        (stateCopy[vehicleID].gps.gpsFix !== "NO GPS" || stateCopy[vehicleID].gps.gpsFix !== "GPS NO FIX") &&
                        // Object.keys(this.connectedVehicles).length === 1)
                        Object.keys(this.vehicles).length === 1
                    ) {
                        stateCopy[vehicleID].isNew = false;
                        // this.setState({mapCenter: [stateCopy[vehiclePosition.vehicleID].position.lat, stateCopy[vehiclePosition.vehicleID].position.lon], mapZoom: 19});
                    }

                    this.vehicles = stateCopy;
                } else if (dataType === "VehicleAttitude") {
                    let vehicleAttitude = jsonData as GlobalTypes.TCPAttitudeType;

                    stateCopy[vehicleID].attitude.roll = vehicleAttitude.roll;
                    stateCopy[vehicleID].attitude.pitch = vehicleAttitude.pitch;
                    stateCopy[vehicleID].attitude.yaw = vehicleAttitude.yaw;

                    stateCopy[vehicleID].updateMarkerAttitude(vehicleAttitude);

                    this.vehicles = stateCopy;
                } else if (dataType === "VehicleAirspeed") {
                    let vehicleAirspeed = jsonData as GlobalTypes.TCPAirspeedType;

                    stateCopy[vehicleID].airspeed = vehicleAirspeed.airspeed;
                    this.vehicles = stateCopy;
                } else if (dataType === "VehicleMission") {
                    let vehicleMission = jsonData as GlobalTypes.TCPMissionType;
                    stateCopy = deepcopy(this.vehicles);
                    stateCopy[vehicleID].setVehicleMission(vehicleMission);
                    this.vehicles = stateCopy;
                } else if (dataType === "VehicleHome") {
                    let vehicleHome = jsonData as GlobalTypes.TCPReturnType & GlobalTypes.MissionItemType;
                    // let stateCopy = deepcopy(this.connectedVehicles);
                    stateCopy = deepcopy(this.vehicles);
                    let tmpHome = {
                        lat: vehicleHome.lat,
                        lon: vehicleHome.lng,
                        alt: vehicleHome.alt
                    };
                    stateCopy[vehicleID].updateHomePosition(tmpHome);
                    this.vehicles = stateCopy;
                } else if (dataType === "VehicleFuel") {
                    let vehicleFuel = jsonData as GlobalTypes.TCPFuelType;

                    stateCopy[vehicleID].fuel.batteryRemaining = vehicleFuel.batteryRemaining;
                    stateCopy[vehicleID].fuel.batteryCurrent = vehicleFuel.batteryCurrent;
                    stateCopy[vehicleID].fuel.batteryVoltage = vehicleFuel.batteryVoltage;

                    this.vehicles = stateCopy;
                } else if (dataType === "VehicleMode") {
                    let vehicleMode = jsonData as GlobalTypes.TCPModeType;
                    stateCopy[vehicleID].vehicleMode = vehicleMode.vehicleMode;
                    this.vehicles = stateCopy;
                } else if (dataType === "VehicleText") {
                    let vehicleText = jsonData as GlobalTypes.TCPTextType;
                    let showMessage = false;
                    // let title = '';
                    // let level = 'info';
                    if (vehicleText.severity === "EMERGENCY") {
                        // title = 'EMERGENCY -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'error';
                        showMessage = this.messagePreferences.emergency;
                    }
                    if (vehicleText.severity === "ALERT") {
                        // title = 'Alert -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'warning';
                        showMessage = this.messagePreferences.alert;
                    }
                    if (vehicleText.severity === "CRITICAL") {
                        // title = 'CRITICAL -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'error';
                        showMessage = this.messagePreferences.critical;
                    }
                    if (vehicleText.severity === "ERROR") {
                        // title = 'ERROR -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'error';
                        showMessage = this.messagePreferences.error;
                    }
                    if (vehicleText.severity === "WARNING") {
                        // title = 'Warning -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'warning';
                        showMessage = this.messagePreferences.warning;
                    }
                    if (vehicleText.severity === "NOTICE") {
                        // title = 'Notice -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'success';
                        showMessage = this.messagePreferences.notice;
                    }
                    if (vehicleText.severity === "INFO") {
                        // title = 'Info -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'info';
                        showMessage = this.messagePreferences.info;
                    }
                    if (vehicleText.severity === "DEBUG") {
                        // title = 'Debug -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'info';
                        showMessage = this.messagePreferences.debug;
                    }

                    if (showMessage) {
                        // this.showNotification(title, vehicleText.text, level, 'bl', 'Got it');
                        stateCopy[vehicleID].messages.unshift({ severity: vehicleText.severity, text: vehicleText.text, timestamp: new Date() });
                        this.vehicles = stateCopy;
                    }
                } else if (dataType === "SensorFootprint") {
                    let jsonFootprint = jsonData as GlobalTypes.TCPSensorFootprintType;
                    stateCopy[vehicleID].sensorFootprint = jsonFootprint.sensorFootprint;
                    this.vehicles = stateCopy;
                } else if (dataType === "VehicleGPS") {
                    let jsonGPS = jsonData as GlobalTypes.TCPGPSType;
                    stateCopy[vehicleID].gps.visibleSats = jsonGPS.visibleSats;
                    stateCopy[vehicleID].gps.gpsFix = jsonGPS.gpsFix;
                    stateCopy[vehicleID].gps.hdop = jsonGPS.hdop;
                    stateCopy[vehicleID].gps.vdop = jsonGPS.vdop;
                    this.vehicles = stateCopy;
                } else if (dataType === "CurrentMissionItem") {
                    let jsonMissionItem = jsonData as GlobalTypes.TCPCurrentMissionItemType;
                    stateCopy[vehicleID].updateCurrentMissionItem(jsonMissionItem.missionItemIndex, false);
                    this.vehicles = stateCopy;
                } else if (dataType === "MissionItemReached") {
                    let jsonMissionItem = jsonData as GlobalTypes.TCPMissionItemReachedType;
                    if (jsonMissionItem.itemIndex === stateCopy[vehicleID].vehicleMission.icons.length - 1) {
                        stateCopy[vehicleID].updateCurrentMissionItem(jsonMissionItem.itemIndex, true);
                    }
                } else if (dataType === "VehicleHeartbeat") {
                    let jsonHeartbeat = jsonData as GlobalTypes.TCPHeartbeatType;
                    stateCopy[vehicleID].general.autopilot = jsonHeartbeat.autopilot;
                    stateCopy[vehicleID].general.commsProtocol = jsonHeartbeat.commsProtocol;
                    stateCopy[vehicleID].general.aircraftType = jsonHeartbeat.aircraftType;
                    stateCopy[vehicleID].general.companion = jsonHeartbeat.companion;
                    stateCopy[vehicleID].general.lastHeard = new Date();
                    stateCopy[vehicleID].setAvailableVehicleModes();
                    this.vehicles = stateCopy;
                } else if (dataType === "VehicleArm") {
                    let jsonArm = jsonData as GlobalTypes.TCPVehicleArmType;
                    stateCopy[vehicleID].isArmed = jsonArm.armed;
                    this.vehicles = stateCopy;
                } else if (dataType === "CurrentVehicleTarget") {
                    let jsonVehicleTarget = jsonData as GlobalTypes.TCPVehicleTargetType;
                    stateCopy[vehicleID].currentTarget.active = true;
                    stateCopy[vehicleID].currentTarget.distanceToTarget = jsonVehicleTarget.distanceToTarget;
                    stateCopy[vehicleID].currentTarget.targetPosition.lat = jsonVehicleTarget.lat;
                    stateCopy[vehicleID].currentTarget.targetPosition.lng = jsonVehicleTarget.lng;
                    stateCopy[vehicleID].currentTarget.targetPosition.alt = jsonVehicleTarget.alt;
                    this.vehicles = stateCopy;
                }
            }
        }
    };
}
