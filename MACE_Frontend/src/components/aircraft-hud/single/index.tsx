import * as React from "react";
import styles from "./styles";
import {
  getFields,
  getDisplayParam,
  getStringValue,
  getBatteryTextColor,
  getTextSeverityColor,
  getShowButton,
} from "../../../util/helpers";
import ReactTooltip from "react-tooltip";
import {
  FaBatteryEmpty,
  FaBatteryQuarter,
  FaBatteryHalf,
  FaBatteryThreeQuarters,
  FaBatteryFull,
  FaLevelUpAlt,
  FaCaretDown,
} from "react-icons/fa";
import {
  FiPlay,
  FiPause,
  FiHome,
  FiCrosshair,
  FiX,
  FiTarget,
  FiCheck,
} from "react-icons/fi";
import { MdFlightTakeoff, MdFlightLand, MdBubbleChart } from "react-icons/md";
import { IoMdSpeedometer } from "react-icons/io";
import colors from "../../../util/colors";
import { LatLng } from "leaflet";
import { AttitudeIndicator, HeadingIndicator } from "react-flight-indicators";
import Select from "react-select";
import SegmentedControl from "../../../common/segmented-control";
import { Context } from "../../../Context";
import * as Types from "../../../data-types/index";
import contextMenu from "../../map/components/context-menu";

const hiddenFields = ["lat", "lng"];

const ROTOR_MODE_OPTIONS = [
  { value: "UNKNOWN", label: "Unknown" },
  { value: "AUTO", label: "Auto" },
  { value: "BRAKE", label: "Brake" },
  { value: "GUIDED", label: "Guided" },
  { value: "LAND", label: "Land" },
  { value: "LOITER", label: "Loiter" },
  { value: "POSHOLD", label: "Position hold" },
  { value: "RTL", label: "Return to Launch" },
  { value: "STABILIZE", label: "Stabilize" },
];

const FIXED_WING_MODE_OPTIONS = [
  { value: "UNKNOWN", label: "Unknown" },
  { value: "AUTO", label: "Auto" },
  { value: "CIRCLE", label: "Circle" },
  { value: "CRUISE", label: "Cruise" },
  { value: "GUIDED", label: "Guided" },
  { value: "INITIALISING", label: "Initialising" },
  { value: "LOITER", label: "Loiter" },
  { value: "MANUAL", label: "Manual" },
  { value: "RTL", label: "Return to launch" },
  { value: "STABILIZE", label: "Stabilize" },
  { value: "TAKEOFF", label: "Takeoff" },
  // { value: "QSTABILIZE", label: "QStabilize" },
  // { value: "QHOVER", label: "QHover" },
  // { value: "QLOITER", label: "QLoiter" },
  // { value: "QLAND", label: "QLand" },
  // { value: "QRTL", label: "QReturn to Launch" },
  // { value: "NR", label: "NR" },
];

const TOGGLE_OPTIONS = [
  { value: "pictoral", label: "Indicators" },
  { value: "text", label: "Text" },
];

type Props = {
  addNotification: (notification: Types.Notification) => void;
  data: Types.Aircraft.AircraftPayload;
  onRequestCenter: (id: string) => void;
  onCommand: (
    command: string,
    filteredAircrafts: Types.Aircraft.AircraftPayload[],
    payload: string[]
  ) => void;
  onUpdateGoHerePts: (point: Types.Vertex & { agentID: string }) => void;
  toggleGoHerePt: (show: boolean, agentID: string) => void;
  target: Types.Vertex;
  defaultAltitude: number;
  onToggleSelect: (agentID: string[]) => void;
  showTargetFlags?: { agentID: string; showGoHere: boolean }[];
  parentScrollPosition: number;
  parentHeight: number;
};

/**
 *  1. Make cross hair center on plane
 *  2. Transform the keys to readable
 *  3. Add ability to toggle
 */

/**
 *  Check to see if any props have changed other than location. Location is not displayed on this UI so no need to re-render if it changes
 */
const excludedKeys = ["location"];
const refMap = {};

const arePropsSame = (prevProps: Props, nextProps: Props) => {
  const container = refMap[nextProps.data.agentID]?.current;
  const windowToScrollAreaOffset = window.innerHeight - nextProps.parentHeight
  const margin = 48 // IDK WHERE THIS COMES FROM TODO?
  let isInViewport = false
  if (
    (container.offsetTop - windowToScrollAreaOffset + margin) <
      nextProps.parentScrollPosition + nextProps.parentHeight 
      &&
      container.offsetTop - windowToScrollAreaOffset + margin + container.clientHeight > nextProps.parentScrollPosition
  ) {
    isInViewport = true
  }

  let same = true;
  const keysToCheck = Object.keys(nextProps.data).filter(
      (k) => excludedKeys.indexOf(k) === -1
  );
  const { data: prevData } = prevProps;
  const { data: nextData } = nextProps;
  keysToCheck.forEach((key) => {
      if (JSON.stringify(prevData[key]) !== JSON.stringify(nextData[key])) {
          same = false;
          return same;
      }
  });
  if (!same) {
    if (isInViewport) {
      return false
    }
    else {
      return true
    }
  }
};

export default React.memo((props: Props) => {
  // console.log("Rendering agent: ", props.data.agentID);
  if (!refMap[props?.data?.agentID]) {
    refMap[props.data.agentID] = React.createRef<HTMLDivElement>();
  }
  const ref = refMap[props?.data?.agentID] || null;

  const fields = getFields(props.data, ["agentID"]);
  const [altitude, setAltitude] = React.useState(props.defaultAltitude);
  const [displayIndicators, setDisplayIndicators] = React.useState("pictoral");
  const [selectedBatteryValue, setSelectedBatteryValue] = React.useState(
    "percent"
  );
  const MODE_OPTIONS =
    fields["vehicle_type"] === "QUADROTOR"
      ? ROTOR_MODE_OPTIONS
      : FIXED_WING_MODE_OPTIONS;
  const ICON_COLOR = colors.gray[700];
  const DISABLED_ICON_COLOR = colors.gray[400];

  const requestCenter = () => {
    const { agentID } = props.data;
    props.onRequestCenter(agentID);
  };

  const updateMission = () => {
    let command: string = "FORCE_DATA_SYNC";
    let payload = [];
    props.onCommand(command, [props.data], payload);
  };

  const arm = (shouldArm: boolean) => {
    let command: string = "SET_VEHICLE_ARM";
    let payload = {
      arm: shouldArm,
    };
    props.onCommand(command, [props.data], [JSON.stringify(payload)]);
  };

  const takeoff = () => {
    let tkoffAlt = props.data.param_list.filter(function (p) {
      return p.param_id === "TKOFF_ALT";
    });
    if (tkoffAlt[0].value >= altitude) {
      let command: string = "TAKEOFF";
      let payload = {
        takeoffPosition: {
          alt: altitude,
        },
        latLonFlag: false,
      };
      props.onCommand(command, [props.data], [JSON.stringify(payload)]);
      updateMission();
    } else {
      props.addNotification({
        title: "Takeoff Failed!",
        message:
          "Takeoff higher than TKOFF_ALT=" +
          tkoffAlt[0].value +
          " for Vehicle " +
          props.data.agentID,
        type: "danger",
      });
    }
  };

  const land = () => {
    let command: string = "LAND";
    let payload = [];
    props.onCommand(command, [props.data], payload);
  };
  const startMission = () => {
    // updateMission();
    // let command: string = "START_MISSION";
    // let payload = [];
    // props.onCommand(command, [props.data], payload);

    // TODO: Figure out smarter way to do mission start. Not intuitive on GUI for UMD, so changing to GUIDED on this button push for now:
    let command: string = "SET_VEHICLE_MODE";
    let payload = {
      mode: "GUIDED",
    };
    props.onCommand(command, [props.data], [JSON.stringify(payload)]);
  };
  const pauseMission = () => {
    // let command: string = "PAUSE_MISSION";
    // let payload = [];
    // props.onCommand(command, [props.data], payload);

    // TODO: Figure out smarter way to do mission pause. Not intuitive on GUI for UMD, so changing to GUIDED on this button push for now:
    let command: string = "SET_VEHICLE_MODE";
    let payload = {
        mode: props.data.vehicle_type === "FIXED_WING" ? "LOITER" : "BRAKE"
    };
    props.onCommand(command, [props.data], [JSON.stringify(payload)]);
  };
  const returnToLaunch = () => {
    let command: string = "RTL";
    let payload = [];
    props.onCommand(command, [props.data], payload);
  };
  const goHere = () => {
    props.toggleGoHerePt(false, props.data.agentID);
    ReactTooltip.hide();
    let command: string = "SET_GO_HERE";
    props.onCommand(command, [props.data], [JSON.stringify(props.target)]);
  };

  const getBatteryIcon = () => {
    if (fields["battery_remaining"] >= 90) {
      return (
        <FaBatteryFull
          style={styles.rotatedIcon}
          color={colors.green[600]}
          size={20}
        />
      );
    } else if (
      fields["battery_remaining"] >= 75 &&
      fields["battery_remaining"] < 90
    ) {
      return (
        <FaBatteryThreeQuarters
          style={styles.rotatedIcon}
          color={colors.green[600]}
          size={20}
        />
      );
    } else if (
      fields["battery_remaining"] < 75 &&
      fields["battery_remaining"] >= 50
    ) {
      return (
        <FaBatteryHalf
          style={styles.rotatedIcon}
          color={colors.yellow[600]}
          size={20}
        />
      );
    } else if (
      fields["battery_remaining"] < 50 &&
      fields["battery_remaining"] >= 25
    ) {
      return (
        <FaBatteryQuarter
          style={styles.rotatedIcon}
          color={colors.orange[600]}
          size={20}
        />
      );
    } else {
      return (
        <FaBatteryEmpty
          style={styles.rotatedIcon}
          color={colors.red[600]}
          size={20}
        />
      );
    }
  };

  const getAircraftText = () => {
    let textColor = colors.gray[500];
    let text = "No messages";

    if (fields["textStr"]) {
      let now = new Date();
      let diff = Math.abs(now.getTime() - fields["textTimestamp"]);
      if (diff > 10000) {
        text = "No new messages";
        textColor = colors.gray[500];
      } else {
        text = fields["textStr"];
        textColor = getTextSeverityColor(fields["textSeverity"]);
      }
    }

    return <span style={styles.label && { color: textColor }}>{text}</span>;
  };

  const setMode = (selectedMode: string) => {
    let command: string = "SET_VEHICLE_MODE";
    let payload = {
      mode: selectedMode["value"],
    };
    props.onCommand(command, [props.data], [JSON.stringify(payload)]);
  };

  const getValueFromMode = () => {
    // console.log(fields["mode"]);
    for (let i = 0; i < MODE_OPTIONS.length; i++) {
      if (fields["mode"] === MODE_OPTIONS[i]["value"]) {
        return MODE_OPTIONS[i];
      }
    }

    return MODE_OPTIONS[0];
  };

  const updateAltitude = (e) => {
    const { name, value } = e.target;
    setAltitude(parseFloat(value));
  };

  const updateTarget = (field: string, value: number) => {
    let newTarget: any = props.target;
    if (field === "lat") {
      newTarget["lat"] = value;
    }
    if (field === "lng") {
      newTarget["lng"] = value;
    }
    if (field === "alt") {
      newTarget["alt"] = value;
    }
    newTarget["agentID"] = props.data.agentID;
    props.onUpdateGoHerePts(newTarget);
  };

  const setTargetToCurrentLocation = (e) => {
    props.toggleGoHerePt(true, props.data.agentID);
    let newTarget: any = props.data.location;
    newTarget["agentID"] = props.data.agentID;
    props.onUpdateGoHerePts(newTarget);
  };

  const getVehicleIcon = () => {
    if (fields["vehicle_type"] === "FIXED_WING") {
      return (
        <img
          src="./icons/fixed-wing.png"
          style={{ height: 20, width: 20 }}
          alt={fields["vehicle_type"]}
        />
      );
    } else {
      return (
        <img
          src="./icons/drone-icon.png"
          style={{ height: 25, width: 25 }}
          alt={fields["vehicle_type"]}
        />
      );
    }
  };

  const getVehicleColor = () => {
    return props.data.color[500];
  };

  const selectAircraft = (
    event: React.MouseEvent<HTMLSpanElement, MouseEvent>
  ) => {
    props.onToggleSelect([props.data.agentID]);
  };

  const showButton = () => {
    return getShowButton(props.data.vehicle_state, props.data.vehicle_type);
  };

  const getBatteryStringValue = () => {
    if (selectedBatteryValue === "percent") {
      return fields["battery_remaining"] + "%";
    } else if (selectedBatteryValue === "voltage") {
      return fields["battery_voltage"] + "V";
    } else {
      return fields["battery_current"] + "A";
    }
  };

  // TODO-PAT: Somehow use the showTargetFlags to disable target button or hide any open tooltips for Go Here for other vehicles...
  // const getShowHUDTooltip = () => {
  //     props.showTargetFlags.forEach(e => {
  //         // console.log(e.showGoHere);
  //         if(e.agentID === props.data.agentID) {
  //             return e.showGoHere;
  //         }
  //     });
  //     return false;
  // }
  return (
    <div style={styles.container} ref={ref}>
      <div
        onClick={(e: React.MouseEvent<HTMLSpanElement, MouseEvent>) =>
          selectAircraft(e)
        }
        style={{ ...styles.header, backgroundColor: getVehicleColor() }}
      >
        <span style={styles.title}>{props.data.agentID}</span>
        {getVehicleIcon()}

        {showButton().arm.show && (
          <button
            style={
              showButton().arm.disabled
                ? styles.disabled_centerButton
                : styles.centerButton
            }
            disabled={showButton().arm.disabled}
            onClick={() => arm(true)}
          >
            {showButton().arm.disabled ? (
              <FiCheck color={colors.gray[400]} size={20} />
            ) : (
              <FiCheck color={colors.teal[100]} size={20} />
            )}
          </button>
        )}
        {showButton().disarm.show && (
          <button
            style={
              showButton().disarm.disabled
                ? styles.disabled_centerButton
                : styles.centerButton
            }
            disabled={showButton().disarm.disabled}
            onClick={() => arm(false)}
          >
            {showButton().disarm.disabled ? (
              <FiX color={colors.gray[400]} size={20} />
            ) : (
              <FiX color={colors.teal[100]} size={20} />
            )}
          </button>
        )}
        <button style={styles.centerButton} onClick={requestCenter}>
          <FiCrosshair color={colors.teal[100]} size={20} />
        </button>
      </div>

      <div key={"mode"} style={Object.assign({}, styles.selectRow)}>
        <Select
          options={MODE_OPTIONS}
          value={getValueFromMode()}
          onChange={(ev) => {
            setMode(ev);
          }}
        />
      </div>

      <div style={styles.hudData}>
        <div key={"behavior"} style={Object.assign({}, styles.hudRow)}>
          <div style={styles.hudElement}>
            <FaLevelUpAlt color={ICON_COLOR} size={20} />
            <span style={styles.hudValue}>
              {getStringValue(fields["alt"].toFixed(2))}
            </span>
          </div>
          <div style={styles.hudElement}>
            <button
              style={styles.centerButton}
              data-for={props.data.agentID + "_battery_tooltip"}
              data-tip="custom show"
              data-event="click"
            >
              {getBatteryIcon()}
              {/* <MdFlightTakeoff data-for={props.data.agentID + "_takeoff_tooltip"} data-tip='custom show' data-event='click' color={ICON_COLOR} size={20} />         */}
              <span
                style={
                  styles.hudValue && {
                    color: getBatteryTextColor(fields["battery_remaining"]),
                  }
                }
              >
                {getBatteryStringValue()}
              </span>
            </button>
            <ReactTooltip
              id={props.data.agentID + "_battery_tooltip"}
              place="left"
              globalEventOff="click"
              clickable={true}
              backgroundColor={colors.white}
            >
              <form>
                <div className="form-check">
                  <label>
                    <input
                      type="radio"
                      name="react-tips"
                      value="percent"
                      checked={selectedBatteryValue === "percent"}
                      className="form-check-input"
                      onChange={() => setSelectedBatteryValue("percent")}
                    />
                    <span style={styles.hudValue}>Percent</span>
                  </label>
                </div>

                <div className="form-check">
                  <label>
                    <input
                      type="radio"
                      name="react-tips"
                      value="voltage"
                      checked={selectedBatteryValue === "voltage"}
                      className="form-check-input"
                      onChange={() => setSelectedBatteryValue("voltage")}
                    />
                    <span style={styles.hudValue}>Voltage</span>
                  </label>
                </div>

                <div className="form-check">
                  <label>
                    <input
                      type="radio"
                      name="react-tips"
                      value="current"
                      checked={selectedBatteryValue === "current"}
                      className="form-check-input"
                      onChange={() => setSelectedBatteryValue("current")}
                    />
                    <span style={styles.hudValue}>Current</span>
                  </label>
                </div>
              </form>
            </ReactTooltip>
          </div>
          <div style={styles.hudElement}>
            <MdBubbleChart color={ICON_COLOR} size={20} />
            <span style={styles.hudValue}>
              {getStringValue(fields["behavior_state"])}
            </span>
          </div>
        </div>
      </div>

      <div style={styles.hudData}>
        <div key={"flight"} style={Object.assign({}, styles.hudRow)}>
          <div style={styles.hudElement}>
            <IoMdSpeedometer color={ICON_COLOR} size={20} />
            <span style={styles.hudValue}>
              {getStringValue(fields["airspeed"].toFixed(2)) + " m/s"}
            </span>
          </div>

          <div style={styles.segmentedControlContainer}>
            <SegmentedControl
              options={TOGGLE_OPTIONS}
              onChange={setDisplayIndicators}
              active={displayIndicators}
            />
          </div>
        </div>
      </div>

      {displayIndicators === "pictoral" && (
        <div style={styles.indicatorContainer}>
          <HeadingIndicator heading={fields["yaw"]} showBox={false} />

          {/* TODO-PAT/AARON: Is negating the roll angle correct here? The roll angle is reported as + for right roll from MACE, this indicator seems to plot the opposite */}
          <AttitudeIndicator
            roll={-fields["roll"]}
            pitch={fields["pitch"]}
            showBox={false}
          />
        </div>
      )}

      {displayIndicators === "text" && (
        <div>
          <div style={styles.hudData}>
            <div key={"roll"} style={Object.assign({}, styles.hudRow)}>
              <div style={styles.hudElement}>
                <span>Roll: </span>
                <span style={styles.hudValue}>
                  {getStringValue(fields["roll"].toFixed(2))}
                </span>
              </div>
              <div style={styles.hudElement}>
                <span>Pitch: </span>
                <span style={styles.hudValue}>
                  {getStringValue(fields["pitch"].toFixed(2))}
                </span>
              </div>
              <div style={styles.hudElement}>
                <span>Yaw: </span>
                <span style={styles.hudValue}>
                  {getStringValue(fields["yaw"].toFixed(2))}
                </span>
              </div>
            </div>
          </div>

          <div style={styles.hudData}>
            <div key={"misc"} style={Object.assign({}, styles.hudRow)}>
              <div style={styles.hudElement}>
                <span>GPS: </span>
                <span style={styles.hudValue}>
                  {getStringValue(fields["gps_fix"])}
                </span>
              </div>
              <div style={styles.hudElement}>
                <span>Distance: </span>
                <span style={styles.hudValue}>
                  {getStringValue(fields["distance_to_target"].toFixed(2))}
                </span>
              </div>
            </div>
          </div>

          <div style={styles.hudData}>
            <div key={"misc"} style={Object.assign({}, styles.hudRow)}>
              <div style={styles.hudElement}>
                <span>Flight time: </span>
                <span style={styles.hudValue}>
                  {getStringValue(fields["flight_time"].toFixed(2))}
                </span>
              </div>
            </div>
          </div>
        </div>
      )}

      <div style={styles.commandsContainer}>
        <div style={styles.commandButtons}>
          {showButton().takeoff.show && (
            <button style={styles.centerButton}>
              <MdFlightTakeoff
                data-for={props.data.agentID + "_takeoff_tooltip"}
                data-tip="custom show"
                data-event="click"
                color={ICON_COLOR}
                size={20}
              />
            </button>
          )}
          {showButton().takeoff.show && (
            <ReactTooltip
              id={props.data.agentID + "_takeoff_tooltip"}
              place="left"
              globalEventOff="click"
              clickable={true}
              backgroundColor={colors.white}
            >
              <div style={styles.singleSettingContainer}>
                <label style={styles.inputLabel}>Takeoff Alt (m):</label>
                <input
                  id="takeoff-tooltip-input"
                  type="number"
                  min={0}
                  defaultValue={altitude}
                  onChange={updateAltitude}
                  name={"alt"}
                  style={styles.input}
                />
              </div>
              <div style={styles.tooltipButtons}>
                <button style={styles.centerButton}>
                  <FiX color={ICON_COLOR} size={20} />
                </button>
                <button style={styles.centerButton} onClick={takeoff}>
                  <MdFlightTakeoff color={ICON_COLOR} size={20} />
                </button>
              </div>
            </ReactTooltip>
          )}
          {showButton().land.show && (
            <button
              style={
                showButton().land.disabled
                  ? styles.disabled_centerButton
                  : styles.centerButton
              }
              disabled={showButton().land.disabled}
              onClick={land}
            >
              {showButton().land.disabled ? (
                <MdFlightLand color={DISABLED_ICON_COLOR} size={20} />
              ) : (
                <MdFlightLand color={ICON_COLOR} size={20} />
              )}
            </button>
          )}
          {showButton().startmission.show && (
            <button
              style={
                showButton().startmission.disabled
                  ? styles.disabled_centerButton
                  : styles.centerButton
              }
              disabled={showButton().startmission.disabled}
              onClick={startMission}
            >
              {showButton().startmission.disabled ? (
                <FiPlay color={DISABLED_ICON_COLOR} size={20} />
              ) : (
                <FiPlay color={ICON_COLOR} size={20} />
              )}
            </button>
          )}
          {showButton().pausemission.show && (
            <button
              style={
                showButton().pausemission.disabled
                  ? styles.disabled_centerButton
                  : styles.centerButton
              }
              disabled={showButton().pausemission.disabled}
              onClick={pauseMission}
            >
              {showButton().pausemission.disabled ? (
                <FiPause color={DISABLED_ICON_COLOR} size={20} />
              ) : (
                <FiPause color={ICON_COLOR} size={20} />
              )}
            </button>
          )}
          {showButton().rtl.show && (
            <button
              style={
                showButton().rtl.disabled
                  ? styles.disabled_centerButton
                  : styles.centerButton
              }
              disabled={showButton().rtl.disabled}
              onClick={returnToLaunch}
            >
              {showButton().rtl.disabled ? (
                <FiHome color={DISABLED_ICON_COLOR} size={20} />
              ) : (
                <FiHome color={ICON_COLOR} size={20} />
              )}
            </button>
          )}

          {showButton().setgohere.show && (
            <button
              style={
                showButton().setgohere.disabled
                  ? styles.disabled_centerButton
                  : styles.centerButton
              }
              disabled={showButton().setgohere.disabled}
              onClick={() => props.toggleGoHerePt(true, props.data.agentID)}
            >
              {showButton().setgohere.disabled ? (
                <FiTarget color={DISABLED_ICON_COLOR} size={20} />
              ) : (
                <FiTarget
                  data-for={"gohere_" + props.data.agentID + "_tooltip"}
                  data-tip="custom show"
                  data-event="click"
                  color={ICON_COLOR}
                  size={20}
                />
              )}
            </button>
          )}
          {showButton().setgohere.show && !showButton().setgohere.disabled && (
            <ReactTooltip
              id={"gohere_" + props.data.agentID + "_tooltip"}
              afterShow={setTargetToCurrentLocation}
              afterHide={() => props.toggleGoHerePt(false, props.data.agentID)}
              place="left"
              clickable={true}
              backgroundColor={colors.white}
            >
              <div style={styles.singleSettingContainer}>
                <label style={styles.inputLabel}>Latitude:</label>
                <input
                  id="latitude-tooltip-input"
                  type="number"
                  value={props.target.lat}
                  onChange={(e) => {
                    const { name, value } = e.target;
                    updateTarget(name, parseFloat(value));
                  }}
                  name={"lat"}
                  style={styles.input}
                />
                <label style={styles.inputLabel}>Longitude:</label>
                <input
                  id="longitude-tooltip-input"
                  type="number"
                  value={props.target.lng}
                  onChange={(e) => {
                    const { name, value } = e.target;
                    updateTarget(name, parseFloat(value));
                  }}
                  name={"lng"}
                  style={styles.input}
                />
                <label style={styles.inputLabel}>Altitude (m):</label>
                <input
                  id="altitude-tooltip-input"
                  type="number"
                  min={altitude}
                  value={props.target.alt}
                  onChange={(e) => {
                    const { name, value } = e.target;
                    updateTarget(name, parseFloat(value));
                  }}
                  name={"alt"}
                  style={styles.input}
                />
              </div>
              <div style={styles.tooltipButtons}>
                <button
                  style={styles.centerButton}
                  onClick={() => {
                    ReactTooltip.hide();
                    props.toggleGoHerePt(false, props.data.agentID);
                  }}
                >
                  <FiX color={ICON_COLOR} size={20} />
                </button>
                <button style={styles.centerButton} onClick={goHere}>
                  <FiCheck color={ICON_COLOR} size={20} />
                </button>
              </div>
            </ReactTooltip>
          )}
        </div>
      </div>

      <div style={styles.row}>{getAircraftText()}</div>
    </div>
  );
}, arePropsSame);
