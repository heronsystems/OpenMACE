import * as L from "leaflet";
import * as React from "react";
import { useLeaflet } from "react-leaflet";
import * as ReactLeaflet from "react-leaflet";
import AppContext, { Context } from "../../../../Context";
import { getCentroid } from "../../../../util/helpers";
import Aircraft from "../aircraft";
import AircraftPath from "../aircraft-path";
import AircraftTarget from "../aircraft-target";
import TargetLine from "../aircraft-target-line";
import Boundary from "../boundary";
import DefaultMarker from "../default-marker";
import EnvironmentIcon from "../environment-icon";
const { useContext } = React;
const { Pane } = ReactLeaflet;

export default () => {
  const { map } = useLeaflet();
  const {
    boundaries,
    aircrafts,
    icons,
    localTargets,
    globalTargets,
    paths,
    updateSelectedAircraft
    
  } = useContext<Context>(AppContext);
  const all = {
    boundaries,
    aircrafts,
    icons,
    localTargets,
    globalTargets,
    paths
  };
  const getPoints = () => {
    const allPoints = [];
    Object.keys(all).map((a) => {
      all[a].map((b) => {
        if (b.location) {
          allPoints.push([b.location.lat, b.location.lng]);
        } else if (b.vertices) {
          b.vertices.map((c) => {
            allPoints.push([c.lat, c.lng]);
          });
        }
      });
    });
    return allPoints;
  };
  const points = getPoints();
  const zoomToPoint = () => {
    const latLngs = points.map((p) => new L.LatLng(p[0], p[1]));
    const markerBounds = L.latLngBounds(latLngs);
    map.fitBounds(markerBounds);
  };
  const getDefaultMarker = () => {
    if (points.length) {
      const center = getCentroid(points);
      return (
        <DefaultMarker
          position={new L.LatLng(center[0], center[1])}
          onClick={zoomToPoint}
        />
      );
    } else {
      return null;
    }
  };
  const CenterMarker = getDefaultMarker();
  const targets = globalTargets.concat(localTargets);
  // console.log("rerendering");
  return (
    <>
      {/* {map.getZoom() < 12 ? (
        CenterMarker
      ) : ( */}
      <>
        <Pane name="aircraft-path" />
        <Pane name="aircraft" />
        {boundaries.map((boundary: Environment.BoundaryPayload, index) => {
          return (
            <Boundary
              key={`boundary-${boundary.boundary_name}`}
              data={boundary}
            />
          );
        })}
        {paths.map((path: Aircraft.PathPayload, index) => {
          return <AircraftPath key={`path-${path.agentID}`} data={path} />;
        })}
        {targets.map((target: Aircraft.TargetPayload, index) => {
          return (
            <AircraftTarget
              key={`target-${target.agentID}-${index}`}
              data={target}
            />
          );
        })}
        {targets.map((target: Aircraft.TargetPayload, index) => {
          return (
            <TargetLine
              key={`target-${target.agentID}-${index}`}
              data={target}
            />
          );
        })}
        {icons.map((icon: Environment.IconPayload) => {
          return (
            <EnvironmentIcon
              key={`environment-icon-${icon.name}`}
              data={icon}
            />
          );
        })}
        {aircrafts.map((aircraft: Aircraft.AircraftPayload, index) => {
          return (
            <Aircraft key={`aircraft-${aircraft.agentID}`} data={aircraft} onToggleSelect = {updateSelectedAircraft} />
          );
        })}
      </>
      {/* )} */}
    </>
  );
};
