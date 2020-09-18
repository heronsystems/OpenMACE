import * as React from "react";
const { useContext } = React;
import AppContext, { Context } from "../../Context";
import AircraftHUD from "./single";
import styles from "./styles";
import { LatLng } from "leaflet";
import { FiSearch, FiX, FiPlay, FiPause, FiHome } from "react-icons/fi";
import { MdFlightTakeoff, MdFlightLand } from "react-icons/md";
import ReactTooltip from "react-tooltip";
import colors from "../../util/colors";
import { Vertex } from "../../data-types";


type Props = {
  onRequestCenter: (LatLng) => void;
  aircrafts: Aircraft.AircraftPayload[];
  onCommand: (command: string, filteredAircrafts: Aircraft.AircraftPayload[], payload: string[]) => void;
  onUpdateGoHerePts: (point: Vertex) => void;
  toggleGoHerePt: (show: boolean) => void;
  target: Vertex;
  defaultAltitude: number;
  onToggleSelect: (agentIDs: string[], show?: boolean) => void;
};

export default React.memo((props: Props) => {
  const [search, setSearch] = React.useState("");
  const [altitude, setAltitude] = React.useState(props.defaultAltitude);
  const [target, setTarget] = React.useState(props.target);
  const [filteredAircrafts, setFilteredAircrafts] = React.useState(
    props.aircrafts
  );
  const [showPaths, setShowPaths] = React.useState(true);

  const GRAY_ICON_COLOR = colors.gray[700];
  const TEAL_ICON_COLOR = colors.teal[100];

  React.useEffect(() => {
    if (search === "") {
      setFilteredAircrafts(props.aircrafts);
    }
  }, [props.aircrafts]);
//   console.log(filteredAircrafts);
//   console.log(props.aircrafts);
  const updateSearch = (value: string) => {
    setSearch(value);
    filterAircrafts(value);
  };
  const filterAircrafts = (searchString: string) => {
    // Make sure we aren't searching for spaces
    const split = searchString.split(",").map((t) => t.replace(/\s/g, ""));
    const aircraftIds = new Set();
    const matches = [];
    // Find all of them matches and add the ID to a set for uniqueness
    split.forEach((term) => {
      const regex = new RegExp(term);
      return props.aircrafts.forEach((a) => {
        if (a.agentID.match(regex)) {
          aircraftIds.add(a.agentID);
        }
      });
    });
    // Loop through the set of unique IDs and get the full aircraft
    aircraftIds.forEach((id) => {
      matches.push(props.aircrafts.find((a) => a.agentID === id));
    });
    setFilteredAircrafts(matches);
  };

  const takeoff = () => {
      let command: string = "TAKEOFF";
      console.log("TODO: Add takeoff altitude to command: " + altitude);
      let payloadArray = [];
      filteredAircrafts.forEach((a) => {
            let payload = {
                takeoffPosition: {
                    alt: altitude
                },
                latLonFlag: false
            };
          payloadArray.push(JSON.stringify(payload));
      });
      props.onCommand(command, filteredAircrafts, payloadArray);
  };
  const land = () => {
      let command: string = "LAND";
      let payload = [];
      props.onCommand(command, filteredAircrafts, payload);
  };
  const startMission = () => {
      let command: string = "START_MISSION";
      let payload = [];
      props.onCommand(command, filteredAircrafts, payload);
      // command = "GET_VEHICLE_MISSION";
      // props.onCommand(command, filteredAircrafts, payload);
  };
  const pauseMission = () => {
      let command: string = "PAUSE_MISSION";
      let payload = [];
      props.onCommand(command, filteredAircrafts, payload);
  };
  const returnToLaunch = () => {
      let command: string = "RTL";
      let payload = [];
      props.onCommand(command, filteredAircrafts, payload);
  };
  const updateAltitude = (e) => {
    const { name, value } = e.target;
    setAltitude(parseFloat(value));
  };
  const toggleShowHidePaths = (event: React.ChangeEvent<HTMLInputElement>) => {
      setShowPaths(!showPaths);
      let agentIDs = [];
      filteredAircrafts.forEach(aircraft => {
          agentIDs.push(aircraft.agentID);
      });
      
      props.onToggleSelect(agentIDs, event.target.checked);
  }

  return props.aircrafts && props.aircrafts.length > 0 ? (
    <div style={styles.container}>
      <div style={styles.searchContainer}>
        <input
          style={styles.searchInput}
          value={search}
          onChange={(e) => updateSearch(e.target.value)}
          placeholder="Filter by Agent ID"
        />
        <FiSearch style={styles.searchIcon} />
        {search !== "" && (
          <FiX style={styles.clearIcon} onClick={() => updateSearch("")} />
        )}
      </div>

      <div style={styles.commandsContainer}>
        <span style={styles.title}>Commands for all filtered agents:</span>
        <div style={styles.commandButtons}>
            <button style={styles.centerButton}>
                <MdFlightTakeoff data-tip='custom show' data-event='click' color={TEAL_ICON_COLOR} size={20} />
            </button>
            <ReactTooltip place='left' globalEventOff='click' clickable={true} backgroundColor={colors.white}>
                <div style={styles.singleSettingContainer}>
                    <label style={styles.inputLabel}>Takeoff Alt (m):</label>
                    <input
                        id="takeoff-tooltip-input"
                        type="number"
                        min={0}
                        defaultValue={10}
                        onChange={updateAltitude}
                        name={"alt"}
                        style={styles.input}
                    />
                </div>
                <div style={styles.tooltipButtons}>
                    <button style={styles.centerButton}>
                        <FiX color={GRAY_ICON_COLOR} size={20} />                        
                    </button>
                    <button style={styles.centerButton} onClick={takeoff}>
                        <MdFlightTakeoff color={GRAY_ICON_COLOR} size={20} />                        
                    </button>
                </div>
            </ReactTooltip>

            <button style={styles.centerButton} onClick={land}>
                <MdFlightLand color={TEAL_ICON_COLOR} size={20} />
            </button>
            <button style={styles.centerButton} onClick={startMission}>
                <FiPlay color={TEAL_ICON_COLOR} size={20} />
            </button>
            <button style={styles.centerButton} onClick={pauseMission}>
                <FiPause color={TEAL_ICON_COLOR} size={20} />
            </button>
            <button style={styles.centerButton} onClick={returnToLaunch}>
                <FiHome color={TEAL_ICON_COLOR} size={20} />
            </button>
        </div>
        <div>
            <input type="checkbox" checked={showPaths} onChange={(event: React.ChangeEvent<HTMLInputElement>) => toggleShowHidePaths(event)} />
            <label style={styles.inputLabel}>Show/hide vehile paths</label>
        </div>
      </div>


      {filteredAircrafts.length ? (
        filteredAircrafts.map((a) => {
          return (
            <AircraftHUD
              data={a}
              onRequestCenter={props.onRequestCenter}
              key={a.agentID}
              onCommand={props.onCommand}
              onUpdateGoHerePts={props.onUpdateGoHerePts}
              toggleGoHerePt = {props.toggleGoHerePt}
              target={props.target}
              defaultAltitude={props.defaultAltitude}
              onToggleSelect={props.onToggleSelect}
            />
          );
        })
      ) : (
        <div style={styles.noResultsContainer}>
          <span style={styles.noResultsText}>No matching vehicles</span>
        </div>
      )}
    </div>
  ) : null;
});
