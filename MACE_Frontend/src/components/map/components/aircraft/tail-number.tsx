import * as React from "react";
import colors from "../../../../util/colors";
import styles from "./styles";

type Props = {
  notification?: boolean;
  value: string;
  color: ColorObject;
};

export default (props: Props) => {
  return (
    <div
      style={Object.assign(
        {},
        styles.tailNumberContainer,
        { backgroundColor: props.color[500], borderColor: props.color[700] },
        props.notification && styles.tailNumberContainer_notification
      )}
    >
      <span style={{ color: props.color[100], fontWeight: 500, fontSize: 11 }}>
        {props.value}
      </span>
    </div>
  );
};
