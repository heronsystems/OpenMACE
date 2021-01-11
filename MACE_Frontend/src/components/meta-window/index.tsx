import * as React from "react";
import styles from "./styles";
import MessageList from "../message-list";
import IndividualAgentUtility from "../glue/individual-agent-utility";
import AgentScheduling from "../glue/agent-scheduling";
const { useContext } = React;
import AppContext, { Context } from "../../Context";

export default () => {
  const {
    GLUESchedules,
    agentUtility,
    totalUtility
  } = useContext<Context>(AppContext);
  return (
    <div style={styles.container}>
      <div style={styles.full}>
        <IndividualAgentUtility
          utility={agentUtility}
          totalUtility={totalUtility}
        />
      </div>
      <div
        style={Object.assign({}, styles.row, {
          marginTop: 40,
          flex: 1,
          maxWidth: "100%"
        })}
      >
        <div style={{ width: "60%" }}>
          <AgentScheduling data={GLUESchedules} />
        </div>
        <div style={{ width: "40%" }}>
          <MessageList />
        </div>
      </div>
    </div>
  );
};
