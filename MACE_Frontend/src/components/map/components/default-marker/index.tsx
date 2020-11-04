import * as React from "react";
import { Marker } from "react-leaflet";
import * as L from "leaflet";
import { renderToString } from "react-dom/server";
import TargetIcon from "./icon";
import * as Types from "../../../../data-types/index";

const DEFAULT_WIDTH = 48;
const DEFAULT_HEIGHT = 48;

type Props = {
  position: Types.Vertex;
  onClick: (e: any) => void;
};

const DefaultMarker = (props: Props) => {
  if (props.position) {
    return (
      <Marker
        position={props.position}
        icon={L.divIcon({
          className: "marker-icon",
          html: renderToString(
            <TargetIcon width={DEFAULT_WIDTH} height={DEFAULT_HEIGHT} />
          ),
          iconSize: L.point(DEFAULT_WIDTH, DEFAULT_HEIGHT, true)
        })}
        onClick={props.onClick}
      ></Marker>
    );
  }
  return null;
};

export default DefaultMarker;
