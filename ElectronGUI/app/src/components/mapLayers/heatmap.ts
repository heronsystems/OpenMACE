require("../../../../html/js/webgl-heatmap.js");
require("../../../../html/js/leaflet-webgl-heatmap.min.js");
// import * as L from 'leaflet';
// import { styles } from "./styles";

/*

TODO-PAT: Figure out how to use leaflet webGLHeatmap plugin (or something better)

*/

export class Heatmap {
    // leafletMap: any;
    // heatmap: any;
    // options: HeatmapOptions
    // dataPoints: number[][];
    // // Leaflet namespace for some reason doesn't have `leafletElement` on the map, which throws compiler errors
    // constructor(leafletMap: any, data?: number[][], options?: HeatmapOptions){
    //     this.leafletMap = leafletMap;
    //     this.options = options ? options : {size: 100, units: 'm', opacity: 0.75, gradientTexture: './images/heatgradient2.png'};
    //     this.dataPoints = data ? data : [];
    //     this.heatmap = L.webGLHeatmap(this.options);
    //     if(this.dataPoints.length > 0) {
    //         this.heatmap.setData(this.dataPoints);
    //         this.show();
    //     }
    // }
    // setData(data: number[][], addToMap: boolean) {
    //     this.dataPoints = data;
    //     this.heatmap.setData(this.dataPoints);
    //     if(addToMap) {
    //         this.show();
    //     }
    // }
    // hide() {
    //     this.leafletMap.leafletElement.removeLayer(this.heatmap);
    // }
    // show() {
    //     this.leafletMap.leafletElement.addLayer(this.heatmap);
    // }
    // setOptions(options: HeatmapOptions) {
    //     this.options = options;
    //     this.heatmap = L.webGLHeatmap(this.options);
    //     this.heatmap.setData(this.dataPoints);
    // }
    // setSize(size: number) {
    //     this.options.size = size;
    //     this.heatmap = L.webGLHeatmap(this.options);
    //     this.heatmap.setData(this.dataPoints);
    // }
    // setUnits(units: 'm' | 'px') {
    //     this.options.units = units;
    //     this.heatmap = L.webGLHeatmap(this.options);
    //     this.heatmap.setData(this.dataPoints);
    // }
    // setOpacity(opacity: number) {
    //     this.options.opacity = opacity;
    //     this.heatmap = L.webGLHeatmap(this.options);
    //     this.heatmap.setData(this.dataPoints);
    // }
    // setGradientTexture(gradientTexture: string) {
    //     this.options.gradientTexture = gradientTexture;
    //     this.heatmap = L.webGLHeatmap(this.options);
    //     this.heatmap.setData(this.dataPoints);
    // }
    // setAlphaRange(range: number) {
    //     this.options.alphaRange = range;
    //     this.heatmap = L.webGLHeatmap(this.options);
    //     this.heatmap.setData(this.dataPoints);
    // }
}
