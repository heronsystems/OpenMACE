import * as React from "react";
import { Marker, Tooltip } from "react-leaflet";
import * as L from "leaflet";
import { renderToString } from "react-dom/server";
import TargetIcon from "./icon";
import styles from "./styles";
import AppContext, { Context } from "../../../../Context";
import colors from "../../../../util/colors";
const { useContext } = React;
import { getTailNumberFromAgentID } from "../../../../util/helpers";
import TailNumber from "./tail-number";
import * as Types from "../../../../data-types/index";

const LOCAL_WIDTH = 24;
const LOCAL_HEIGHT = 24;

const GLOBAL_WIDTH = 48;
const GLOBAL_HEIGHT = 48;

type Props = {
  data: Types.Aircraft.TargetPayload;
};

export default React.memo((props: Props) => {
  const { aircrafts } = useContext<Context>(AppContext);
  const aircraft = aircrafts.find((a) => a.agentID === props.data.agentID);
  const scope = props.data.is_global ? "global" : "local";
  const width = props.data.is_global ? GLOBAL_WIDTH : LOCAL_WIDTH;
  const height = props.data.is_global ? GLOBAL_HEIGHT : LOCAL_HEIGHT;
  const centeredLocation = { lat: props.data.location.lat - 0.00005, lng: props.data.location.lng };
  if (props.data.agentID === undefined) {
    // console.log("Rerendering target as " + props.data.should_display);
    if (props.data.should_display === true) {
      return (
        <Marker
          position={centeredLocation}
          icon={L.divIcon({
            className: "target-icon",
            html: renderToString(
              <>
                <TargetIcon
                  scope={scope}
                  color={colors.gray}
                  width={width}
                  height={height}
                  selected={true}
                />

              </>
            ),
            iconSize: L.point(width, height, true),
            iconAnchor: L.point(width / 2, height, true)
          })}
        ></Marker>
      );
    } else {
      return null;
    }
  } else {
    const tailNumber = getTailNumberFromAgentID(props.data.agentID.toString());
    return (
      <Marker
        position={centeredLocation}
        icon={L.divIcon({
          className: "target-icon",
          html: renderToString(
            <>
              <TargetIcon
                scope={scope}
                color={aircraft.color}
                width={width}
                height={height}
                selected={aircraft.selected}
              />
              <TailNumber
                value={tailNumber}
                color={aircraft.selected ? aircraft.color[500] : aircraft.color[500] + "30"}
              />
            </>
          ),
          iconSize: L.point(width, height, true),
          iconAnchor: L.point(width / 2, height, true)
        })}
      >
        {/* {!props.data.is_global && (
          <Tooltip
            direction="right"
            offset={[16, -height / 2]}
            opacity={1}
            className="marker-tooltip"
            permanent={props.data.is_global}
          >
            <div style={styles.tooltip}>
              <div style={styles.row}>
                <span style={styles.label}>Agent:</span>
                <span style={styles.value}>{props.data.agentID}</span>
              </div>
            </div>
          </Tooltip>
        )} */}
      </Marker>
    );
  }
});
