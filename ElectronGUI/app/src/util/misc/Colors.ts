import * as colors from "material-ui/styles/colors"

export type ColorTypes =
    | { Primary: "#FF7A00" }
    | { Secondary: "#FF9839" }
    | { Tertiary: "#0E2C85" }
    | { Quaternary: "#1339AC" }
    | { Quinary: "#03899C" }
    | { Success: "#2DC748" }
    | { Error: "#FF1300" }
    | { Warning: "#FFAE00" }
    | { Text: "#111111" }
    | { Underline: "#d4d4d4" }
export const Colors = {
    Primary: "#FF7A00" as "#FF7A00",
    Secondary: "#FF9839",
    Tertiary: "#0E2C85",
    Quaternary: "#1339AC",
    Quinary: "#03899C",
    Success: "#2DC748",
    Error: "#FF1300",
    Warning: "#FFAE00",
    Text: "#111111",
    Underline: "#d4d4d4"
}

// TODO: Figure out a better way to do background colors
// export const backgroundColors = ['rgba(255,0,0,1)', 'rgba(0,0,255,1)', 'rgba(0,0,0,1)', 'rgba(0,255,0,1)', 'rgba(255,255,0,1)', 'rgba(255,153,0,1)'];

// export const opaqueBackgroundColors = ['rgba(255,0,0,0.2)', 'rgba(0,0,255,0.2)', 'rgba(0,0,0,0.2)', 'rgba(0,255,0,0.2)', 'rgba(255,255,0,0.2)', 'rgba(255,153,0,0.2)'];

export function getRandomRGB() {
    let rgb = {
        r: Math.floor(Math.random() * 256),
        g: Math.floor(Math.random() * 256),
        b: Math.floor(Math.random() * 256)
    }

    return rgb
}

export function textSeverityToColor(severity: string) {
    let color = colors.black
    if (severity === "EMERGENCY") {
        color = colors.redA700
    } else if (severity === "ALERT") {
        color = colors.amber500
    } else if (severity === "CRITICAL") {
        color = colors.orange500
    } else if (severity === "ERROR") {
        color = colors.red500
    } else if (severity === "WARNING") {
        color = colors.deepOrange500
    } else if (severity === "NOTICE") {
        color = colors.deepPurple500
    } else if (severity === "INFO") {
        color = colors.blue500
    } else if (severity === "DEBUG") {
        color = colors.green500
    } else {
        color = colors.black
    }

    return color
}
