import * as React from "react";
import { Tooltip } from "react-leaflet";
import { PointTuple } from "leaflet";
import styles from "./styles";
import * as Types from "../../../../data-types/index";

type Props = {
  data: Types.Environment.IconPayload;
  direction: "right" | "left" | "top" | "bottom";
  offset: PointTuple;
  show?: boolean;
};

export default (props: Props) => {
  if (props.show) {
    return (
      <Tooltip
        direction="right"
        offset={props.offset}
        opacity={1}
        className="marker-tooltip"
      >
        <div style={styles.tooltip}>
          <div style={styles.row}>
            <span style={styles.label}>Name:</span>
            <span style={styles.value}>{props.data.name}</span>
          </div>
          <div style={styles.row}>
            <span style={styles.label}>Latitude:</span>
            <span style={styles.value}>{props.data.lat}</span>
          </div>
          <div style={styles.row}>
            <span style={styles.label}>Longitude:</span>
            <span style={styles.value}>{props.data.lng}</span>
          </div>
        </div>
      </Tooltip>
    );
  }
  return null;
};
