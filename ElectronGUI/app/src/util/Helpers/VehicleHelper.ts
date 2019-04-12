export function aircraftImgSrcFromType(acType: string) {
    let filename = "images/drone-icon.png";
    if (acType === "GENERIC") {
        filename = "";
    } else if (acType === "HELICOPTER") {
        filename = "./images/heli-icon.png";
    } else if (acType === "GCS") {
        filename = "";
    } else if (acType === "REPEATER") {
        filename = "";
    } else if (acType === "GROUND_ROVER") {
        filename = "";
    } else if (acType === "SURFACE_BOAT") {
        filename = "";
    } else if (acType === "TRICOPTER") {
        filename = "";
    } else if (acType === "QUADROTOR") {
        filename = "./images/drone-icon.png";
    } else if (acType === "HEXAROTOR") {
        filename = "";
    } else if (acType === "OCTOROTOR") {
        filename = "";
    } else if (acType === "ONBOARD_CONTROLLER") {
        filename = "";
    } else if (acType === "FIXED_WING") {
        filename = "./images/fixed-wing.png";
    }

    return filename;
}
