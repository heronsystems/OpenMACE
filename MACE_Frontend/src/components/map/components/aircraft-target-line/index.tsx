import * as React from "react";
import { Marker, Tooltip, Polyline } from "react-leaflet";
import * as L from "leaflet";
import { renderToString } from "react-dom/server";
import AppContext, { Context } from "../../../../Context";
import colors from "../../../../util/colors";
const { useContext } = React;
import { getTailNumberFromAgentID, getColorForIndex } from "../../../../util/helpers";
import { MdInvertColors } from "react-icons/md";
import * as Types from "../../../../data-types/index";


type Props = {
  data: Types.Aircraft.TargetPayload;
};

let lastUpdate = Date.now()

// const checkIfTimeToUpdate = () => {
//   let preventUpdate = true
//   const now = Date.now()
//   const diff = now - lastUpdate
//   if (diff > 100) {
//     preventUpdate = false
//     lastUpdate = now
//   }
//   return preventUpdate
// };

const TargetLine = React.memo((props: Props) => {
  const { aircrafts } = useContext<Context>(AppContext);
  const aircraft = aircrafts.find((a) => a.agentID === props.data.agentID);
  if (aircraft === undefined) {
    return null;
  } else{
    return (
      <Polyline
        positions={[props.data.location, aircraft ? aircraft.location : props.data.location]}
        color={aircraft.selected ? aircraft.color[500] : aircraft.color[500] + "30"}
        weight={5}
      />

    );
  }
// }, checkIfTimeToUpdate);
});


export default TargetLine;