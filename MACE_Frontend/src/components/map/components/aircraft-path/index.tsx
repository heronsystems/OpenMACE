import * as React from "react";
import { Polyline } from "react-leaflet";
import colors from "../../../../util/colors";
import AppContext, { Context } from "../../../../Context";
import * as Types from "../../../../data-types/index";
const { useContext } = React;

type Props = {
  data: Types.Aircraft.PathPayload;
};

export default (props: Props) => {

  const { aircrafts } = useContext<Context>(AppContext);
  const aircraft = aircrafts.find((a) => a.agentID === props.data.agentID);
  if (aircraft) {
  }
  return (
    <>
      <Polyline
        pane="aircraft-path"
        positions={props.data.vertices}
        weight={3}
        dashArray="10"
        lineCap={"round"}
        color={aircraft.selected ? aircraft.color[500] : aircraft.color[500] + "30"}
      />
    </>
  );
};
