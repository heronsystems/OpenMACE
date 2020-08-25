import * as React from "react";
import styles from "./styles";

type Props = {
  children?: any;
};

export default (props: Props) => {
  return (
    <div style={styles.absoluteContainer}>
      <div style={styles.container}>
        <div style={styles.innerContainer}>{props.children}</div>
      </div>
    </div>
  );
};
