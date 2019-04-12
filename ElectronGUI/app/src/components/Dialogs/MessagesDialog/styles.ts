import * as colors from "material-ui/styles/colors"
import { textSeverityToColor } from "../../../util/misc/Colors"

export const styles = {
    checkboxStyle: {
        marginBottom: 16
    },
    labelStyle: {
        color: colors.orange700
    },
    dialogTitle: {
        backgroundColor: colors.orange700,
        color: colors.white
    },
    gridStyle: {
        marginTop: 35
    },
    checkboxLabel_Emergency: {
        color: textSeverityToColor("EMERGENCY")
    },
    checkboxLabel_Critical: {
        color: textSeverityToColor("CRITICAL")
    },
    checkboxLabel_Alert: {
        color: textSeverityToColor("ALERT")
    },
    checkboxLabel_Error: {
        color: textSeverityToColor("ERROR")
    },
    checkboxLabel_Warning: {
        color: textSeverityToColor("WARNING")
    },
    checkboxLabel_Notice: {
        color: textSeverityToColor("NOTICE")
    },
    checkboxLabel_Info: {
        color: textSeverityToColor("INFO")
    },
    checkboxLabel_Debug: {
        color: textSeverityToColor("DEBUG")
    }
}
