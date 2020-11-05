import * as React from "react";
import { Marker, Popup } from "react-leaflet";
import * as L from "leaflet";
import { renderToString } from "react-dom/server";
import HomeIcon from "./icon";
import Tooltip from "../tooltip";
const { useContext, useState, useEffect, useRef } = React;
import AppContext, { Context } from "../../../../../Context";
import styles from "./styles";
import * as Types from "../../../../../data-types/index";

const DEFAULT_WIDTH = 48;
const DEFAULT_HEIGHT = 48;

type Props = {
  data: Types.Environment.IconPayload;
};

const HomeMarker = (props: Props) => {
  const _marker: React.RefObject<Marker> = useRef();
  const { icons, updateIcons, aircrafts  } = useContext<Context>(AppContext);
  const aircraft = aircrafts.find((a) => a.agentID === props.data.agentID);
  
  return (
    <Marker
      ref={_marker}
      position={{lat: props.data.lat, lng: props.data.lng}}
      icon={L.divIcon({
        className: "home-icon",
        html: renderToString(
          <HomeIcon 
            width={DEFAULT_WIDTH} 
           height={DEFAULT_HEIGHT}
           color={aircraft.color} />
        ),
        iconSize: L.point(DEFAULT_WIDTH, DEFAULT_HEIGHT, true)
      })}
      // @ts-ignore this exists?
    >
      <Tooltip
        direction="right"
        offset={[DEFAULT_WIDTH / 4, 0]}
        data={props.data}
      />
     
    </Marker> 
  );
};

export default HomeMarker;
