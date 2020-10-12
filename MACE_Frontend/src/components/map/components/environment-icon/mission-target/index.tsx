import * as React from "react";
import { Marker } from "react-leaflet";
import * as L from "leaflet";
import { renderToString } from "react-dom/server";
import TargetIcon from "./icon";
import Tooltip from "../tooltip";
import * as Types from "../../../../../data-types/index";

const DEFAULT_WIDTH = 48;
const DEFAULT_HEIGHT = 48;

type Props = {
  data: Types.Environment.IconPayload;
};

const TargetMarker = (props: Props) => {
  return (
    <Marker
      position={{lat: props.data.lat, lng: props.data.lng}}
      icon={L.divIcon({
        className: "mission-target-icon",
        html: renderToString(
          <TargetIcon width={DEFAULT_WIDTH} height={DEFAULT_HEIGHT} />
        ),
        iconSize: L.point(DEFAULT_WIDTH, DEFAULT_HEIGHT, true)
      })}
    >
      <Tooltip
        direction="right"
        offset={[DEFAULT_WIDTH / 4, 0]}
        data={props.data}
        show
      />
    </Marker>
  );
};

export default TargetMarker;
