import * as deepcopy from "deepcopy";
const turf = require("@turf/turf");
// var geometryHelper = require('leaflet-geometryutil');
import * as L from "leaflet";
import * as GlobalTypes from "../../types/globalTypings";

export class PolygonHelper {
    drawPolygonPts: GlobalTypes.PositionType[];
    gridPts?: { inPoly: L.LatLng[]; trimmedPts: L.LatLng[] };
    showEnvironmentSettings?: boolean;
    environmentSettings?: GlobalTypes.EnvironmentSettingsType;
    showDraw?: boolean;
    envBoundingBox?: GlobalTypes.PositionType[];
    globalOrigin: GlobalTypes.PositionType;

    constructor() {
        this.drawPolygonPts = [];
        this.gridPts = { inPoly: [], trimmedPts: [] };
        this.showEnvironmentSettings = false;
        this.environmentSettings = { minSliderVal: 25, maxSliderVal: 100, showBoundingBox: false, gridSpacing: -1 };
        this.showDraw = false;
        this.envBoundingBox = [];
        this.globalOrigin = { lat: 0, lng: 0, alt: 0 };
    }

    handleAddPolygonPt = (e: L.LeafletMouseEvent) => {
        if (this.showDraw) {
            let tmpPts = this.drawPolygonPts;

            // Make sure the new point is not causing an intersection with the existing polygon:
            let intersection = false;
            if (this.drawPolygonPts.length > 2) {
                let prevPt = this.drawPolygonPts[this.drawPolygonPts.length - 1];
                let newLine = turf.lineString([[prevPt.lat, prevPt.lng], [e.latlng.lat, e.latlng.lng]]);
                for (let i = 1; i < this.drawPolygonPts.length - 1; i++) {
                    let pt1 = [this.drawPolygonPts[i - 1].lat, this.drawPolygonPts[i - 1].lng];
                    let pt2 = [this.drawPolygonPts[i].lat, this.drawPolygonPts[i].lng];
                    let tmpLine = turf.lineString([pt1, pt2]);
                    let intersects = turf.lineIntersect(tmpLine, newLine);

                    if (intersects.features[0]) {
                        intersection = true;
                    }
                }
            }

            if (!intersection) {
                tmpPts.push({ lat: e.latlng.lat, lng: e.latlng.lng, alt: 0 });
                this.drawPolygonPts = tmpPts;
                this.updateGrid();
            } else {
                // let title = 'Draw boundary';
                // let level = 'warning';
                // this.showNotification(title, 'Segment cannot intersect boundary.', level, 'tc', 'Got it');
            }
        }
    };

    handleDeleteLastPolygonPt = () => {
        let tmpPts = this.drawPolygonPts;
        tmpPts.pop();
        this.drawPolygonPts = tmpPts;
        this.updateGrid();
    };

    handleDisableDraw = () => {
        this.showDraw = false;
        this.drawPolygonPts = [];
        this.gridPts = { inPoly: [], trimmedPts: [] };
        // this.pauseMACEComms = false;
    };

    handleSubmitBoundary = () => {
        // TODO:
        //  1) Send environment boundary to MACE

        // TODO: Send to MACE:

        if (this.drawPolygonPts.length > 2) {
            this.showDraw = false;
            this.drawPolygonPts = [];
            // this.makeTCPRequest(0, "SET_ENVIRONMENT_VERTICES", JSON.stringify({boundary: this.drawPolygonPts}));
        } else {
            // let title = 'Draw boundary';
            // let level = 'info';
            // this.showNotification(title, 'Boundary must have 3 or more vertices to be valid', level, 'tc', 'Got it');
        }
    };

    handleClearPts = () => {
        this.drawPolygonPts = [];
        this.gridPts = { inPoly: [], trimmedPts: [] };
    };

    handleChangeGridSpacing = (val: number) => {
        let settings = deepcopy(this.environmentSettings);
        settings.gridSpacing = val;
        this.environmentSettings = settings;
        // this.updateGrid();
    };

    updateGrid = () => {
        // let coordinatesArr: any = [];
        // this.drawPolygonPts.forEach(function(coord: GlobalTypes.PositionType) {
        //     coordinatesArr.push([coord.lat, coord.lng]);
        // });
        // let geoJsonData: GeoJSON.GeoJsonObject = {
        //     type: "Feature",
        //     bbox: coordinatesArr
        // };
        // let geoJsonLayer = L.geoJSON(geoJsonData);
        // let bounds: L.LatLngBounds = geoJsonLayer.getBounds();
        // let boundingBox: GlobalTypes.PositionType[] = [];
        // boundingBox.push({lat: bounds.getSouthWest().lng, lng: bounds.getSouthWest().lat, alt: 0}); // Bottom Left
        // boundingBox.push({lat: bounds.getSouthWest().lng, lng: bounds.getNorthEast().lat, alt: 0}); // Bottom Right
        // boundingBox.push({lat: bounds.getNorthEast().lng, lng: bounds.getNorthEast().lat, alt: 0}); // Top Right
        // boundingBox.push({lat: bounds.getNorthEast().lng, lng: bounds.getSouthWest().lat, alt: 0}); // Top Left
        // this.envBoundingBox = boundingBox;
        // Calculate grid lines based on global origin:
        // this.calculateGridPts(boundingBox);
    };

    calculateGridPts = (boundingBox: GlobalTypes.PositionType[]) => {
        // // Only if lat/lng are not at the origin and the grid spacing is greater than 0
        // if(this.globalOrigin.lat !== 0 && this.globalOrigin.lng !== 0) {
        // if(this.environmentSettings.gridSpacing > 0) {
        //     let bottomLeft = new L.LatLng(boundingBox[0].lat, boundingBox[0].lng);
        //     let bottomRight = new L.LatLng(boundingBox[1].lat, boundingBox[1].lng);
        //     // let topRight = new L.LatLng(boundingBox[2].lat, boundingBox[2].lng);
        //     let topLeft = new L.LatLng(boundingBox[3].lat, boundingBox[3].lng);
        //     let horizDistance = geometryHelper.length([bottomLeft, bottomRight]); // distance between bottom two points
        //     let vertDistance = geometryHelper.length([bottomLeft, topLeft]); // distance between two left points
        //     let distanceToNextPt = this.environmentSettings.gridSpacing;
        //     let prevPt = bottomLeft;
        //     let tmpGridPts: L.LatLng[] = [];
        //     let tmpTrimmedPts: L.LatLng[] = [];
        //     let numXPts = Math.round(horizDistance/distanceToNextPt);
        //     let numYPts = Math.round(vertDistance/distanceToNextPt);
        //     for(let i = 0; i <= numYPts; i++) {
        //     // Add previous point to the array:
        //     if(this.isPtInPoly(prevPt, this.drawPolygonPts)) {
        //         tmpGridPts.push(prevPt);
        //     }
        //     else {
        //         tmpTrimmedPts.push(prevPt);
        //     }
        //     let tmpNewPt = prevPt;
        //     for(let j = 0; j <= numXPts; j++) {
        //         // Move East to the next point and add to the map:
        //         tmpNewPt = geometryHelper.destination(tmpNewPt, 90, distanceToNextPt);;
        //         // Check if in the polygon or not:
        //         if(this.isPtInPoly(tmpNewPt, this.drawPolygonPts)) {
        //         tmpGridPts.push(tmpNewPt);
        //         }
        //         else {
        //         tmpTrimmedPts.push(tmpNewPt);
        //         }
        //     }
        //     // Move North to the next point:
        //     prevPt = geometryHelper.destination(prevPt, 0, distanceToNextPt);;
        //     }
        //     // Set the grid points to display:
        //     let pts = {inPoly: tmpGridPts, trimmedPts: tmpTrimmedPts};
        //     this.gridPts = pts;
        // }
        // }
    };

    isPtInPoly = (marker: L.LatLng, boundary: GlobalTypes.PositionType[]): boolean => {
        let polyPoints: L.LatLng[] = [];
        for (let i = 0; i < boundary.length; i++) {
            polyPoints.push(new L.LatLng(boundary[i].lat, boundary[i].lng));
        }
        let x = marker.lat;
        let y = marker.lng;

        let inside = false;
        for (let i = 0, j = polyPoints.length - 1; i < polyPoints.length; j = i++) {
            let xi = polyPoints[i].lat,
                yi = polyPoints[i].lng;
            let xj = polyPoints[j].lat,
                yj = polyPoints[j].lng;

            let intersect = yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi;
            if (intersect) inside = !inside;
        }

        return inside;
    };

    saveEnvironmentSettings = (settings: GlobalTypes.EnvironmentSettingsType) => {
        if (settings.gridSpacing >= settings.minSliderVal && settings.gridSpacing <= settings.maxSliderVal) {
            console.log("Settings: " + JSON.stringify(settings));
            this.environmentSettings = settings;
            // this.updateGrid();
        } else {
            // let title = 'Environment settings';
            // let level = 'info';
            // this.showNotification(title, 'Grid spacing must be within the minimum and maximum values.', level, 'tc', 'Got it');
        }
    };
}
