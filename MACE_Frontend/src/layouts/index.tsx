import * as React from "react";
const { version } = require("../../package.json");
import styles from "./styles";

type Props = {
  children: any;
};

export default (props: Props) => {
  return (
    <div style={{ height: "100%", width: "100%" }}>
      {props.children}
      <div style={styles.version}>
        <span style={styles.versionText}>
          Version: <span style={styles.versionNumber}>{version}</span>
        </span>
      </div>
    </div>
  );
};
