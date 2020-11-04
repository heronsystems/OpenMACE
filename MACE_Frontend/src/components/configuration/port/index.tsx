import * as React from "react";
import Modal from "react-modal";
import styles from "./styles";

Modal.setAppElement("#app");

type PortSettings = {
  [port_type: string]: number;
};

type Props = {
  onSave: (settings: PortSettings) => void;
  defaultPorts: PortSettings;
  open?: boolean;
  onRequestClose: () => void;
};
const customStyles = {
  content: {
    top: "50%",
    left: "50%",
    right: "auto",
    bottom: "auto",
    marginRight: "-50%",
    transform: "translate(-50%, -50%)",
    minWidth: "50%"
  },
  overlay: {
    position: "fixed",
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: "rgba(0, 0, 0, 0.45)"
  }
};

export default (props: Props) => {
  const [ports, setPorts] = React.useState({ ...props.defaultPorts });
  const updatePort = (e) => {
    const { name, value } = e.target;
    ports[name] = parseInt(value);
    setPorts(ports);
  };
  const resetInputsAndClose = () => {
    setPorts(props.defaultPorts);
    props.onRequestClose();
  };
  const saveAndClose = () => {
    props.onSave(ports);
    props.onRequestClose();
  };
  const getLabelForPort = (port: string) => {
    switch (port) {
      case "map_port":
        return "Map Port";
      default:
        return port;
    }
  };
  return (
    <div>
      <Modal
        isOpen={props.open}
        onRequestClose={resetInputsAndClose}
        style={customStyles}
      >
        <div style={styles.header}>
          <span style={styles.heading}>Port Configuration</span>
        </div>
        <div style={styles.inputs}>
          {Object.keys(ports).map((p) => {
            return (
              <div style={styles.singleSettingContainer} key={p}>
                <label style={styles.label}>{getLabelForPort(p)}</label>
                <input
                  type="text"
                  defaultValue={ports[p]}
                  onChange={updatePort}
                  name={p}
                  style={styles.input}
                />
              </div>
            );
          })}
        </div>
        <div style={styles.buttonContainer}>
          <button style={styles.cancelButton} onClick={resetInputsAndClose}>
            Cancel
          </button>
          <button style={styles.saveButton} onClick={saveAndClose}>
            Save
          </button>
        </div>
      </Modal>
    </div>
  );
};
