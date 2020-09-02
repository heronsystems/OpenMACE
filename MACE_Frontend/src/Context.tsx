import * as React from "react";
const { createContext } = React;
const AppContext = createContext(null);
const { Consumer, Provider } = AppContext;
import { State } from "./Provider";

export type Context = State & {
  updateIcons: (icon: Environment.IconPayload) => void;
  updateTargets: (target: Aircraft.TargetPayload) => void;
  removeIcon: (name: string) => void;
  setGlobalZoom: (zoom: number) => void;
  sendToMACE: (command: string, filteredAircrafts: Aircraft.AircraftPayload[], payload: string[]) => void;
  updateSelectedAircraft: (agentIDs: string[], show?: boolean) => void;
};

const withAppContext = (Component) => {
  return function WrapperComponent(props) {
    return (
      <Consumer>
        {(context: Context) => <Component {...props} context={context} />}
      </Consumer>
    );
  };
};

export { Consumer, Provider, withAppContext };
export default AppContext;
