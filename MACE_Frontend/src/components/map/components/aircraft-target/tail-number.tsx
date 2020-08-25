import * as React from "react";
import styles from "./styles";

type Props = {
  value: string;
  color: string;
};

export default (props: Props) => {
  return (
    <div style={styles.tailNumberContainer}>
      <span style={{...styles.tailNumber, color : props.color}}>{props.value}</span>
    </div>
  );
};
