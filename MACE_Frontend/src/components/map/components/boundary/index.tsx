import * as React from "react";
import { Polygon } from "react-leaflet";
import colors from "../../../../util/colors";
import * as Types from "../../../../data-types/index";

type Props = {
  data: Types.Environment.BoundaryPayload;
};

export default (props: Props) => {
  const { boundary_type, vertices } = props.data;
  return (
    <Polygon
      color={boundary_type === "hard" ? colors.red[600] : colors.yellow[500]}
      positions={vertices}
      fillOpacity={0}
    />
  );
};
