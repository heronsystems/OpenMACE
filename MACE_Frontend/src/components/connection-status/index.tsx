import * as React from "react";
const { useContext } = React;
import AppContext, { Context } from "../../Context";
import { FiCloudOff } from "react-icons/fi";
import styles from "./styles";
import ReactLoading from "react-loading";
import colors from "../../util/colors";

type Props = {};

export default (props: Props) => {
  const { connectionState } = useContext<Context>(AppContext);
  if (connectionState === "connected") {
    return null;
  }
  return (
    <div style={styles.container}>
      <div style={styles.content}>
        <FiCloudOff style={styles.icon} />
        <span style={styles.message}>
          You are not connected to the data source
        </span>
      </div>
      <div style={styles.ctaContainer}>
        {connectionState === "disconnected" ? (
          <button style={Object.assign({}, styles.button)} onClick={() => {}}>
            Retry
          </button>
        ) : (
          <ReactLoading
            type="spin"
            color={colors.blue[500]}
            width={32}
            height={32}
          />
        )}
      </div>
    </div>
  );
};
