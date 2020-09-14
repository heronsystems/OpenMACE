import {
  getCentroid,
  formatPoints,
  capitalize,
  getColorForIndex,
  orderColorsByWeight,
  getAgent,
  flattenObject,
  getFields,
  splitMessageString,
  areObjectsSame
} from "./helpers";

describe("Test the helpers provided by helpers.ts", () => {
  // Object.defineProperty(window, "require", {
  //   writable: true,
  //   value: jest.fn().mockImplementation(query => ({
  //     matches: false,
  //     media: query,
  //     onchange: null,
  //     addListener: jest.fn(), // deprecated
  //     removeListener: jest.fn(), // deprecated
  //     addEventListener: jest.fn(),
  //     removeEventListener: jest.fn(),
  //     dispatchEvent: jest.fn()
  //   }))
  // });
  // console.log(window.require);
  it("Should convert the points correctly from {lat: number, lng:number}[] to [lat, lng][]", () => {
    const vertices = [
      { lat: 38.32724845095056, lng: -76.77091598510744 },
      { lat: 38.32670978823017, lng: -76.57281875610353 },
      { lat: 38.29519105238491, lng: -76.53470993041994 },
      { lat: 38.27187993253012, lng: -76.56372070312501 },
      { lat: 38.26972361264482, lng: -76.76061630249025 }
    ];
    expect(formatPoints(vertices)).toStrictEqual([
      [38.32724845095056, -76.77091598510744],
      [38.32670978823017, -76.57281875610353],
      [38.29519105238491, -76.53470993041994],
      [38.27187993253012, -76.56372070312501],
      [38.26972361264482, -76.760616302490253]
    ]);
  });
  it("Should pass the [lat, lng] value through as it's the expected format for formatPoints", () => {
    const vertices = [
      [38.32724845095056, -76.77091598510744],
      [38.32670978823017, -76.57281875610353],
      [38.29519105238491, -76.53470993041994],
      [38.27187993253012, -76.56372070312501],
      [38.26972361264482, -76.760616302490253]
    ];
    expect(formatPoints(vertices)).toStrictEqual([
      [38.32724845095056, -76.77091598510744],
      [38.32670978823017, -76.57281875610353],
      [38.29519105238491, -76.53470993041994],
      [38.27187993253012, -76.56372070312501],
      [38.26972361264482, -76.760616302490253]
    ]);
  });
  it("Should capitalize first letter of string", () => {
    const str = "capitalize";
    expect(capitalize(str)).toBe("Capitalize");
  });
  it("Should get a random color for an index", () => {
    const color = getColorForIndex(0);
    expect(color).toBe("#feb2b2");
  });
  it("Should get all of the colors by weight", () => {
    const colors = orderColorsByWeight();
    expect(colors).toMatchObject([
      "100",
      "200",
      "300",
      "400",
      "500",
      "600",
      "700",
      "800",
      "900"
    ]);
  });
  it("Should test the agent simulation", () => {
    const agent = getAgent();
    expect(agent.length).toBeTruthy();
  });
  it("Should flatten the object", () => {
    const obj = {
      agentID: "test",
      fidelity: "real",
      autonomy_state: "",
      communication_state: "transmitting",
      vehicle_state: "",
      type: "fast",
      behavior_state: "",
      remediation_state: false,
      orientation: {
        pitch: 0,
        roll: 0,
        yaw: 0
      },
      location: { lat: 38.28454701883166, lng: -76.66688919067384 }
    };
    const flat = flattenObject(obj);
    expect(Object.keys(flat).length).toBe(13);
  });
  it("Should get the flattened object w/o excluded fields", () => {
    const obj = {
      agentID: "test",
      fidelity: "real",
      autonomy_state: "",
      communication_state: "transmitting",
      vehicle_state: "",
      type: "fast",
      behavior_state: "",
      remediation_state: false,
      orientation: {
        pitch: 0,
        roll: 0,
        yaw: 0
      },
      location: { lat: 38.28454701883166, lng: -76.66688919067384 }
    };
    const fields = getFields(obj, ["agentID", "fidelity"]);
    expect(Object.keys(fields).length).toBe(11);
  });

  it("Should return that the objects are the same", () => {
    const a = {
      id: 1,
      value: "24",
      location: {
        lat: null
      }
    };
    const b = {
      id: 1,
      value: "24",
      location: {
        lat: null
      }
    };
    expect(areObjectsSame(a, b)).toBeTruthy();
  });

  it("Should return if the separator is found", () => {
    const message = '{name: 1, value: "test"}\r';
    const message_separator = "\r";
    let message_separater_index = message.indexOf(message_separator);
    let foundEntireMessage = message_separater_index !== -1;
    expect(foundEntireMessage).toBeTruthy();
  });

  it("Should return if the separator is found", () => {
    const message = '{name: 1, value: "test"}';
    const message_separator = "\r";
    let message_separater_index = message.indexOf(message_separator);
    let foundEntireMessage = message_separater_index !== -1;
    expect(foundEntireMessage).toBeFalsy();
  });
});
