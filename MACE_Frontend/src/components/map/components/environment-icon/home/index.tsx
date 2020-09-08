import * as React from "react";
import { Marker, Popup } from "react-leaflet";
import * as L from "leaflet";
import { renderToString } from "react-dom/server";
import HomeIcon from "./icon";
import Tooltip from "../tooltip";
const { useContext, useState, useEffect, useRef } = React;
import AppContext, { Context } from "../../../../../Context";
import styles from "./styles";

const DEFAULT_WIDTH = 48;
const DEFAULT_HEIGHT = 48;

type Props = {
  data: Environment.IconPayload;
};

const HomeMarker = (props: Props) => {
  const _marker: React.RefObject<Marker> = useRef();
  // const _popup: React.RefObject<Popup> = useRef();
  // const [longitude, setLongitude] = useState(props.data.lng);
  // const [latitude, setLatitude] = useState(props.data.lat);
  const { icons, updateIcons, removeIcon } = useContext<Context>(AppContext);
  const updateIconPosition = ({ lat, lng }) => {
    const existing = icons.find((i) => i.name === props.data.name);
    updateIcons({
      ...existing,
      lat,
      lng,
      auto_focus: false
    });
  };
  // const updateTempPosition = ({ lat, lng }) => {
  //   setLatitude(lat);
  //   setLongitude(lng);
  // };
  // const save = () => {
  //   updateIconPosition({ lat: latitude, lng: longitude });
  //   _marker.current.leafletElement.closePopup();
  //   // TODO: Send event to HMI with save details
  // };
  // const reset = () => {
  //   setLatitude(props.data.lat);
  //   setLongitude(props.data.lng);
  //   _marker.current.leafletElement.closePopup();
  //   _marker.current.leafletElement.setLatLng({lat: props.data.lat, lng: props.data.lng});
  // };
  // const onDragEnd = (e) => {
  //   updateTempPosition(e.target.getLatLng());
  //   _marker.current.leafletElement.togglePopup();
  // };
  // const handleMarkerClick = () => {
  // };
  // useEffect(() => {
  //   if (props.data.auto_focus) {
  //     _marker.current.leafletElement.openPopup();
  //   }
  // }, [{lat: props.data.lat, lng: props.data.lng}]);
  // const remove = () => {
  //   removeIcon(props.data.name);
  //   // TODO: Send event to HMI w/ removal details
  // };
  return (
    <Marker
      ref={_marker}
      position={{lat: props.data.lat, lng: props.data.lng}}
      icon={L.divIcon({
        className: "home-icon",
        html: renderToString(
          <HomeIcon width={DEFAULT_WIDTH} height={DEFAULT_HEIGHT} />
        ),
        iconSize: L.point(DEFAULT_WIDTH, DEFAULT_HEIGHT, true)
      })}
      // @ts-ignore this exists?
     
    >
      <Tooltip
        direction="right"
        offset={[DEFAULT_WIDTH / 4, 0]}
        data={props.data}
      />
      {/* <Popup
        autoClose={false}
        closeOnClick={false}
        offset={new L.Point(0, -16)}
        ref={_popup}
      >
        <>
          <div>Drag marker or enter coordinates</div>
          <div style={styles.manualEntryContainer}>
            <div style={styles.inputContainer}>
              <label>Latitude</label>
              <input
                style={styles.input}
                value={latitude}
                onChange={(e) => setLatitude(e.target.value as any)}
              />
            </div>
            <div style={styles.inputContainer}>
              <label>Longitude</label>
              <input
                style={styles.input}
                value={longitude}
                onChange={(e) => setLongitude(e.target.value as any)}
              />
            </div>
          </div>
        </>
        <div style={styles.actionsContainer}>
          <button style={styles.saveButton} onClick={save}>
            Save
          </button>
          <button style={styles.cancelButton} onClick={reset}>
            Cancel
          </button>
          <button style={styles.removeButton} onClick={remove}>
            Remove
          </button>
        </div>
      </Popup>*/}
    </Marker> 
  );
};

export default HomeMarker;
