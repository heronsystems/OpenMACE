import * as L from "leaflet";
import colors from "./colors";
import * as _ from "lodash";
import * as Types from "../data-types/index";

type SimpleLocation = { lat: number; lng: number };
type LocationArray = number[][];

const TOTAL_AMOUNT_OF_ITEMS_IN_PLOT = 22;

export const formatPoints = (
  arr: LocationArray | SimpleLocation[]
): LocationArray => {
  if (typeof arr[0] === "object") {
    if (arr[0].hasOwnProperty("lat") && arr[0].hasOwnProperty("lng")) {
      return (arr as SimpleLocation[]).map((location) => {
        return [location.lat, location.lng];
      });
    } else {
      return arr as LocationArray;
    }
  }
};

export const getCentroid = (arr: LocationArray | SimpleLocation[]) => {
  const points: LocationArray = formatPoints(arr);
  return points.reduce(
    function (x, y) {
      return [x[0] + y[0] / arr.length, x[1] + y[1] / arr.length];
    },
    [0, 0]
  );
};

const isCollinear = (p1, p2, p3) => {
  return (p1.y - p2.y) * (p1.x - p3.x) == (p1.y - p3.y) * (p1.x - p2.x);
};

const moveTo = (b, a, r) => {
  const vector = { x: b.x - a.x, y: b.y - a.y };
  const length = Math.sqrt(vector.x * vector.x + vector.y * vector.y);
  const unitVector = { x: vector.x / length, y: vector.y / length };
  return { x: a.x + unitVector.x * r, y: a.y + unitVector.y * r };
};

export const smoothPolyline = (
  coordinates: { lat: number; lng: number }[],
  radius: number
) => {
  const points = coordinates.map((p) => ({ x: p.lat, y: p.lng }));
  points
    .slice(1)
    .reduce(
      (acc, p, i, points) => {
        let next = points[i + 1];
        let prev = acc[acc.length - 1];

        if (next && !isCollinear(prev.point, p, next)) {
          let before = moveTo(prev.point, p, radius);
          let after = moveTo(next, p, radius);
          return acc.concat({
            point: p,
            s: `L ${before.x} ${before.y} S ${p.x} ${p.y} ${after.x} ${after.y} `
          });
        } else {
          return acc.concat({
            point: p,
            s: `L ${p.x} ${p.y} `
          });
        }
      },
      [
        {
          point: points[0],
          s: `M ${points[0].x} ${points[0].y} `
        }
      ]
    )
    .map((p) => p.s)
    .join("");
  return points.map((p) => ({ lat: p.x, lng: p.y }));
};

export const animateMarker = (
  leafletElement: any,
  coordFrom: L.LatLng,
  coordTo: L.LatLng
): void => {
  let position;
  let movingInterval;
  let curLat = coordFrom.lat;
  let curLng = coordFrom.lng;
  let newLat = coordTo.lat;
  let newLng = coordTo.lng;
  let threshold = 0.01;
  movingInterval = setInterval(() => {
    if (threshold >= 1) clearInterval(movingInterval);
    position = new L.LatLng(
      curLat + (newLat - curLat) * threshold,
      curLng + (newLng - curLng) * threshold
    );
    leafletElement.setLatLng(position);
    threshold += 0.1;
  }, 400);
};

export const capitalize = (s: string) => {
  if (typeof s !== "string") return "";
  return s.charAt(0).toUpperCase() + s.slice(1);
};

export const getEventNameFromType = (type: Types.MessageType) => {
  switch (type) {
    case "environment_boundary":
      return "Environment Boundary";
    case "environment_icon":
      return "Environment Icon";
    case "vehicle_heartbeat":
      return "Vehicle Heartbeat";
    case "vehicle_path":
      return "Vehicle Path";
    case "vehicle_target":
      return "Vehicle Target";
    case "vehicle_parameter_list":
        return "Vehicle Parameter List";
    default:
      console.warn(`Could not find conversion for ${type}`);
  }
};

export const getDisplayParam = (param: string) => {
  switch (param) {
    case "mode":
      return "Mode";
    case "behavior_state":
      return "Behavior State";
    case "vehicle_state":
      return "Vehicle State";
    case "pitch":
      return "Pitch";
    case "roll":
      return "Roll";
    case "yaw":
      return "Yaw";
    case "lat":
      return "Latitude";
    case "lng":
      return "Longitude";
    case "alt":
      return "Altitude";
    default:
      // console.warn(`No conversion for ${param}`);
      return param;
  }
};

export const getStringValue = (val: string | boolean) => {
  if (typeof val === "boolean") {
    return val.toString();
  }

  if(val === "") {
      return "UNKNOWN";
  }
  return val;
};

export const getBatteryTextColor = (val: number) => {
    if(val >= 75) {
        return colors.green[600];
    }
    else if(val < 75 && val >= 50) {
        return colors.yellow[600];
    }
    else if(val < 50 && val >= 25) {
        return colors.orange[600];
    }
    else {
        return colors.red[600];
    }
}

export const getTextSeverityColor = (severity: string) => {
    if(severity === "ERROR") {
        return colors.red[600];
    }
    else if(severity === "WARNING") {
        return colors.yellow[700]
    }
    else if(severity === "INFO") {
        return colors.blue[700]
    }
    else {
        return colors.gray[500]
    }
}

const getOrderedColors = () => {
  const excluded = ["gray"];
  return Object.keys(colors)
    .map((key) => {
      if (typeof colors[key] === "object" && excluded.indexOf(key) === -1) {
        return Object.keys(colors[key]).map((weight) => {
          if (parseInt(weight) >= 400) {
            return colors[key][weight];
          }
        });
      }
    })
    .reduce((acc, val) => acc.concat(val), [])
    .filter((c) => c);
};

export const orderColorsByWeight = (
  excludedColors: string[] = [],
  excludedWeights: string[] = []
) => {
  const set = new Set();
  Object.keys(colors).forEach((key) => {
    if (
      typeof colors[key] === "object" &&
      excludedColors.indexOf(colors[key]) === -1
    ) {
      Object.keys(colors[key]).forEach((weight) => {
        if (excludedWeights.indexOf(weight) === -1) {
          set.add(weight);
        }
      });
    }
  });
  return Array.from(set);
};

const getColorNames = () => {
  return Object.keys(colors).map((color) => color);
};

const getColorWeights = () => {
  const set = new Set();
  Object.keys(colors).forEach((key) => {
    if (typeof colors[key] === "object") {
      Object.keys(colors[key]).forEach((weight) => {
        set.add(weight);
      });
    }
  });
  return Array.from(set) as string[];
};

export const getColorForIndex = (index: number) => {
  const excludedColors = ["gray"];
  const excludedWeights = ["100", "200"];
  const colorNames = getColorNames().filter(
    (c) => excludedColors.indexOf(c) === -1
  );
  const colorWeights = getColorWeights().filter(
    (w) => excludedWeights.indexOf(w) === -1
  );
  const availableColors = [];
  colorWeights.forEach((weight) => {
    return colorNames.forEach((color) => {
      if (typeof colors[color] === "object") {
        availableColors.push(colors[color][weight]);
      }
    });
  });
  return availableColors[index];
};

export const getAgent = () => {
  const num = Math.round(Math.random() * 2);
  if (num === 0) {
    return ["agent-a"];
  } else if (num === 1) {
    return ["agent-b"];
  } else if (num === 2) {
    return ["agent-a", "agent-b"];
  }
};


export const radiansToDegrees = (radians: number) => {
  const pi = Math.PI;
  return radians * (180 / pi);
};

/**
 *  Flattens an object to one level
 * @param obj
 * @param preserveHierarchy If true, preserves hierarchy with period separators i.e. orientation.yaw, location.lat
 */

export const flattenObject = (
  obj: Object,
  preserveHierarchy: boolean = false
) => {
  const toReturn = {};

  for (let i in obj) {
    if (!obj.hasOwnProperty(i)) continue;

    if (typeof obj[i] === "object") {
      const flatObject = flattenObject(obj[i]);
      for (let x in flatObject) {
        if (!flatObject.hasOwnProperty(x)) continue;
        if (preserveHierarchy) {
          toReturn[i + "." + x] = flatObject[x];
        } else {
          toReturn[x] = flatObject[x];
        }
      }
    } else {
      toReturn[i] = obj[i];
    }
  }
  return toReturn;
};

export const getFields = (obj: Object, exclude: string[] = []) => {
  const rtnObject = {};
  const flat = flattenObject(obj);
  Object.keys(flat)
    .filter((key) => {
      return exclude.indexOf(key) === -1;
    })
    .forEach((k) => {
      rtnObject[k] = flat[k];
    });
  return rtnObject;
};

export function fmtMSS(s) {
  return (s - (s %= 60)) / 60 + (9 < s ? ":" : ":0") + s;
}

export function sToHMS(s) {
  return new Date(s * 1000).toISOString().substr(11, 8);
}

/**
 *  This is necessary due to bundled messages coming from HMI.
 *  If the JSON.parse() fails, then it's likely that we have an invalid JSON string with several messages.
 */
export const splitMessageString = (str: string) => {
  // Remove all line breaks
  str = str.replace(/(\r\n|\n|\r)/gm, "");
  // Remove whitespace
  str = str.replace(/\s/g, "");
  const regex = new RegExp(/}{/gm);
  // Split at the new object & make sure to add the opening & closing bracket as they may have been removed in the split
  const arr = str.split(regex).map((a) => {
    if (a[0] !== "{") {
      a = "{" + a;
    }
    if (a[a.length - 1] !== "}") {
      a = a + "}";
    }
    return a;
  });
  return arr;
};

/**
 * Parses a string containing one or multiple JSON encoded objects in the string.
 * The result is always an array of objects.
 *
 * @param  {String} data
 * @return {Array}
 */
export function parseJson(data) {
  data = data.replace("\n", "", "g");

  var start = data.indexOf("{"),
    open = 0,
    i = start,
    len = data.length,
    result = [];

  for (; i < len; i++) {
    if (data[i] == "{") {
      open++;
    } else if (data[i] == "}") {
      open--;
      if (open === 0) {
        result.push(JSON.parse(data.substring(start, i + 1)));
        start = i + 1;
      }
    }
  }

  return result;
}

export const areObjectsSame = (a: Object, b: Object) => {
//   console.log(a, b);
//   console.log(JSON.stringify(a) === JSON.stringify(b))
//   console.log(JSON.stringify(a), JSON.stringify(b))
//   return JSON.stringify(a) === JSON.stringify(b);
    return _.isEqual(a, b);
};

export const getTailNumberFromAgentID = (agentID: string) => {
  if (!agentID) {
    return null;
  }
  const split = agentID.split("_");
  if (split.length > 1) {
    return split[1];
  } else {
    return split[0];
  }
};

export const checkIfEqual = (
  a: Object,
  b: Object,
  excludedKeys: string[] = []
) => {
  let equal = true;
  const keysA = Object.keys(a);
  const keysB = Object.keys(b);
  if (JSON.stringify(keysA) !== JSON.stringify(keysB)) {
    return false;
  }
  const keysToCheck = keysA.filter((k) => excludedKeys.indexOf(k) === -1);
  keysToCheck.forEach((key) => {
    if (JSON.stringify(a[key]) !== JSON.stringify(b[key])) {
      equal = false;
      return equal;
    }
  });
  return equal;
};

const defaultButtonStatus = {
    "takeoff": {
        show: false,
        disabled: true
    },
    "land": {
        show: false,
        disabled: true
    },
    "startmission": {
        show: false,
        disabled: true
    },
    "pausemission": {
        show: false,
        disabled: true
    },
    "rtl": {
        show: false,
        disabled: true
    },
    "setgohere": {
        show: false,
        disabled: true
    },
    "arm": {
        show: true,
        disabled: false
    },
    "disarm": {
        show: false,
        disabled: true
    }
}

export const getShowButton = (vehicleState: string, vehicleType: string) => {
    let buttons = defaultButtonStatus;
    // console.log(vehicleState);
    switch (vehicleState) {
        case "Grounded":
        case "Grounded Idle":
        case "Grounded Disarmed":
            buttons.takeoff = { show: true, disabled: false};
            buttons.land = { show: false, disabled: vehicleType === "FIXED_WING" ? true : true};
            buttons.startmission = { show: true, disabled: true};
            buttons.pausemission = { show: true, disabled: true};
            buttons.rtl = { show: true, disabled: true};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: true, disabled: false};
            buttons.disarm = { show: false, disabled: true};
            break;
        case "Grounded Armed":
            buttons.takeoff = { show: true, disabled: false};
            buttons.land = { show: false, disabled: vehicleType === "FIXED_WING" ? true : true};
            buttons.startmission = { show: true, disabled: true};
            buttons.pausemission = { show: true, disabled: true};
            buttons.rtl = { show: true, disabled: true};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: false};
            break;
        case "Grounded Arming":
        case "Grounded Disarming":
            buttons.takeoff = { show: true, disabled: false};
            buttons.land = { show: false, disabled: vehicleType === "FIXED_WING" ? true : true};
            buttons.startmission = { show: true, disabled: true};
            buttons.pausemission = { show: true, disabled: true};
            buttons.rtl = { show: true, disabled: true};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: true};
            break;
        case "Flight":
        case "Flight Takeoff":
        case "Flight Takeoff Climbing":
        case "Flight Takeoff Transitioning":
            buttons.takeoff = { show: false, disabled: true};
            buttons.land = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.startmission = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.pausemission = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.rtl = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: true};
            break;
        case "Flight Manual":
            buttons.takeoff = { show: false, disabled: true};
            buttons.land = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.startmission = { show: true, disabled: false};
            buttons.pausemission = { show: true, disabled: false};
            buttons.rtl = { show: true, disabled: false};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: true};
            break;
        case "Flight Takeoff Complete":
        case "Flight Guided":
        case "Flight Guided Idle":
        case "Flight Guided Spatial Item":
        case "Flight Guided Queue":
        case "Flight Guided AttTarget":
        case "Flight Guided GeoTarget":
        case "Flight Guided CartTarget":
            buttons.takeoff = { show: false, disabled: true };
            buttons.land = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false };
            buttons.startmission = { show: true, disabled: false };
            buttons.pausemission = { show: true, disabled: false};
            buttons.rtl = { show: true, disabled: false };
            buttons.setgohere = { show: true, disabled: false };

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: true};
        break;
        case "Flight Brake":
        case "Flight Loiter":
        case "Flight Unknown":
            buttons.takeoff = { show: false, disabled: true};
            buttons.land = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.startmission = { show: true, disabled: false};
            buttons.pausemission = { show: true, disabled: false};
            buttons.rtl = { show: true, disabled: false};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: true};
            break;
        case "Flight Auto":
            buttons.takeoff = { show: false, disabled: true};
            buttons.land = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.startmission = { show: true, disabled: true};
            buttons.pausemission = { show: true, disabled: false};
            buttons.rtl = { show: true, disabled: false};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: true};
            break;
        case "Flight RTL":
            buttons.takeoff = { show: false, disabled: true};
            buttons.land = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.startmission = { show: true, disabled: true};
            buttons.pausemission = { show: true, disabled: false};
            buttons.rtl = { show: true, disabled: true};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: true};
            break;
        case "Flight Land":
        case "Flight Landing":
        case "Flight Landing Transitioning":
        case "Flight Landing Descent":
            buttons.takeoff = { show: false, disabled: true};
            buttons.land = { show: true, disabled: vehicleType === "FIXED_WING" ? true : true};
            buttons.startmission = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.pausemission = { show: true, disabled: vehicleType === "FIXED_WING" ? true : false};
            buttons.rtl = { show: true, disabled: true};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: true};
            break;
        case "Flight Landing Complete":
            buttons.takeoff = { show: true, disabled: false};
            buttons.land = { show: false, disabled: vehicleType === "FIXED_WING" ? true : true};
            buttons.startmission = { show: true, disabled: true};
            buttons.pausemission = { show: false, disabled: true};
            buttons.rtl = { show: true, disabled: true};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: false, disabled: true};
            buttons.disarm = { show: true, disabled: true};
            break;
        case "Unknown":
            buttons.takeoff = { show: true, disabled: true};
            buttons.land = { show: false, disabled: vehicleType === "FIXED_WING" ? true : true};
            buttons.startmission = { show: true, disabled: true};
            buttons.pausemission = { show: false, disabled: true};
            buttons.rtl = { show: true, disabled: true};
            buttons.setgohere = { show: true, disabled: true};

            buttons.arm = { show: true, disabled: false};
            buttons.disarm = { show: false, disabled: true};
            break;

        default:
            break;
    }

    return buttons;
}



export const getDisplayTargetAndPath = (mode: string) => {
    if(mode === "GUIDED" 
       || mode === "CIRCLE"
       || mode === "RTL"
       || mode === "LOITER"
       || mode === "TAKEOFF"
       || mode === "AUTO") 
       {
           return true;
       }
       else {
           return false;
       }
}


export const pickAircraftColor = (numAircraft: number): Types.ColorObject => {
    // let numAircraft = this._aircraft_list._aircrafts.length;
    while (numAircraft > 9) {
      numAircraft-=10;
    }
    switch (numAircraft) {
      case 0:
        return colors.blue;
      case 1:
        return colors.green;
      case 2:
        return colors.pink;
      case 3:
        return colors.orange;
      case 4:
        return colors.purple;
      case 5:
        return colors.red;
      case 6:
        return colors.yellow;
      case 7:
        return colors.teal;
      case 8:
        return colors.indigo;
      case 9:
        return colors.gray;
    }
  }

export const constructDefaultAircraft = (numAircraft: number): Types.Aircraft.AircraftPayload => {
    // TODO-PAT: Commented out for spamming testing:
  // this.sendToMACE("GET_ENVIRONMENT_BOUNDARY",[],[]);

  let now = new Date();
  let timestamp = now.getTime();
  return {
      agentID: "DEFAULT",
      selected: true,
      behavior_state: "",
      vehicle_state: "UNINITIALIZED",
      vehicle_type: "QUADROTOR",
      color: pickAircraftColor(numAircraft),
      orientation: {
          pitch: 0.0,
          roll: 0.0,
          yaw: 0.0
      },
      location: {
          lat: 0.0,
          lng: 0.0,
          alt: 0.0
      },
      armed: false,
      visible_sats: 0.0,
      gps_fix: "",
      hdop: 0.0,
      vdop: 0.0,
      text: {
          textStr: "No messages",
          textSeverity: "",
          textTimestamp: timestamp
      },
      mode: "",
      battery_remaining: 0.0,
      battery_current: 0.0,
      battery_voltage: 0.0,
      param_list: [
          {
              param_id: "TKOFF_ALT",
              value: 50
          }
      ],
      airspeed: 0.0,
      distance_to_target: 0.0,
      flight_time: 0.0,
      lastUpdate: Date.now()
  }
}