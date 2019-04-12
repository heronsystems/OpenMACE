import getMuiTheme from "material-ui/styles/getMuiTheme"
const lightMuiTheme = getMuiTheme()
import * as colors from "material-ui/styles/colors"

export const styles = {
    labelStyle: {
        color: colors.orange700
    },
    dialogTitle: {
        backgroundColor: colors.orange700,
        color: colors.white
    },
    dialogContent: {
        width: "20%"
    },
    floatingLabelFocus: {
        color: colors.orange700
    },
    underlineFolcusStyle: {
        borderColor: colors.orange700
    },
    selectField: {
        marginRight: 10,
        width: "100%",
        backgroundColor: lightMuiTheme.palette.canvasColor
    }
}
