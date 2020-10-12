import * as React from "react";
import styles from "./styles";
import onClickOutside from "react-onclickoutside";
import * as Types from "../../../../data-types/index";

type Props = {
  position: { x: number; y: number };
  visible: boolean;
  onRequestClose: () => void;
  actions?: { label: string; action: () => void }[];
};
const ContextMenu = (props: Props) => {
  /// @ts-ignore this works, TODO: Bobby figure out correct typings
  ContextMenu.handleClickOutside = () => props.onRequestClose();
  if (props.visible) {
    return (
      <div
        style={Object.assign({}, styles.container, {
          left: props.position.x,
          top: props.position.y,
        })}
      >
        {props.actions.map((action, index) => {
          return (
            <div
              key={index}
              style={styles.option}
              onClick={() => {
                action.action();
                props.onRequestClose();
              }}
            >
              {action.label}
            </div>
          );
        })}
      </div>
    );
  } else return null;
};
const clickOutsideConfig = {
  /// @ts-ignore this works, TODO: Bobby figure out correct typings
  handleClickOutside: () => ContextMenu.handleClickOutside,
};
export default onClickOutside(ContextMenu, clickOutsideConfig);