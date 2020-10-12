import * as React from "react";
import CommandControl from "./command-control";
import MissionTarget from "./mission-target";
import Home from "./home";
import Origin from "./origin";
import * as Types from "../../../../data-types/index";

type Props = {
  data: Types.Environment.IconPayload;
};

export default (props: Props) => {
  switch (props.data.type) {
    case "command_control":
      return <CommandControl {...props} />;
    case "mission_target":
      return <MissionTarget {...props} />;
    case "takeoff_land":
        return <Home {...props} />;
    case "origin":
      return <Origin {...props} />;
    default:
      return null;
  }
};
