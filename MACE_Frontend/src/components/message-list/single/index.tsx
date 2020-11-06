import * as React from "react";
const { useState } = React;
import styles from "./styles";
import { Collapse } from "react-collapse";
import { getEventNameFromType } from "../../../util/helpers";
import { FiChevronDown } from "react-icons/fi";
import moment from "moment";

type Props = {
  data: Message;
};

export default (props: Props) => {
  const [open, setOpen] = useState(false);
  const toggle = () => {
    setOpen(!open);
  };
  const getAllDetails = () => {
    const details = [];
    Object.keys(props.data).map(k => {
      if (typeof props.data[k] === "string") {
        details.push({ label: k, value: props.data[k] });
      } else if (typeof props.data[k] === "object") {
        Object.keys(props.data[k]).map(l => {
          if (typeof props.data[k][l] === "string") {
            details.push({ label: l, value: props.data[k][l] });
          } else if (typeof props.data[k][l] === "object") {
            details.push({ label: l, value: JSON.stringify(props.data[k][l]) });
          }
        });
      }
    });
    return details;
  };
  return (
    <div style={styles.row}>
      <div style={styles.content}>
        <span style={styles.rowTitle}>
          {getEventNameFromType(props.data.message_type as any)}
        </span>
        <span style={styles.date}>
          {moment(props.data.date).format("HH:mm:ss")}
        </span>
      </div>
      <span style={styles.expandText} onClick={toggle}>
        MORE DETAILS
        <FiChevronDown
          style={{ marginLeft: 8, position: "relative", top: 2 }}
        />
      </span>
      <Collapse isOpened={open}>
        <ul style={styles.detailList}>
          {getAllDetails().map((d, i) => (
            <li key={i} style={styles.detailRow}>
              <span style={styles.label}>{d.label}</span>
              <span style={styles.value}>{d.value}</span>
            </li>
          ))}
        </ul>
      </Collapse>
    </div>
  );
};
