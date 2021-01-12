import * as React from "react";
import Single from "./single";
import styles from "./styles";

type Props = {
  data: GLUE.AssignmentPayload[];
};

export default (props: Props) => {
  return (
    <div style={styles.outerContainer}>
      <p style={styles.heading}>Agent Scheduling</p>
      <div style={styles.container}>
        {props.data.length ? (
          props.data.map((d, index) => {
            return <Single key={d.agentID} data={d} index={index} />;
          })
        ) : (
          <span style={styles.noAgentsText}>No Agents Available</span>
        )}
      </div>
    </div>
  );
};
