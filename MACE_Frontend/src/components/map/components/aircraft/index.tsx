import * as React from "react";
import { Marker } from "react-leaflet";
import * as L from "leaflet";
import { renderToString } from "react-dom/server";
import QuadIcon from "./quad_icon";
import FixedWingIcon from "./fixed_wing_icon";
import { getTailNumberFromAgentID } from "../../../../util/helpers";
import colors from "../../../../util/colors";
import TailNumber from "./tail-number";
import * as Types from "../../../../data-types/index";

const DEFAULT_WIDTH = 44;
const DEFAULT_HEIGHT = 44;

type Props = {
  data: Types.Aircraft.AircraftPayload;
  onToggleSelect: (agentID: string[]) => void;
};

const lastUpdateObj = {}

// const checkIfTimeToUpdate = (props: Props) => {
//   const {agentID} = props.data
//   let preventUpdate = true
//   const now = Date.now()
//   if (!lastUpdateObj[agentID]) {
//     lastUpdateObj[agentID] = Date.now()
//     preventUpdate = false
//   }
//   else {
//     const diff = now - lastUpdateObj[agentID]
//     if (diff > 100) {
//       preventUpdate = false
//       lastUpdateObj[agentID] = Date.now()
//     }
//   }
//   return preventUpdate
// };

export default React.memo((props: Props) => {
  const selectAircraft = (event: React.MouseEvent<HTMLSpanElement, MouseEvent>) => {
    props.onToggleSelect([props.data.agentID]);
  }

  const tailNumber = getTailNumberFromAgentID(props.data.agentID.toString());
  const color = props.data.color;
//   const altScale = 1 + 0.01*props.data.location.alt;
  const altScale = 1;
  return (
    <Marker
      pane="aircraft"
      position={props.data.location}
      onClick={(e: React.MouseEvent<HTMLSpanElement, MouseEvent>) => selectAircraft(e)}
      icon={L.divIcon({
        className: "aircraft-icon",
        html: renderToString(
          <>
            {props.data.vehicle_type === "QUADROTOR" ?
                <QuadIcon
                width={DEFAULT_WIDTH * altScale}
                height={DEFAULT_HEIGHT * altScale}
                rotation={props.data.orientation.yaw}
                color={color}
                />
                :
                <FixedWingIcon
                width={DEFAULT_WIDTH * altScale}
                height={DEFAULT_HEIGHT * altScale}
                rotation={props.data.orientation.yaw}
                color={color}
                />
            }
            <TailNumber
              // notification={true}
              value={tailNumber}
              color={color}
            />
          </>
        ),
        iconSize: L.point(DEFAULT_WIDTH, DEFAULT_HEIGHT, true)
      })}
    />
  );
// }, checkIfTimeToUpdate);
});