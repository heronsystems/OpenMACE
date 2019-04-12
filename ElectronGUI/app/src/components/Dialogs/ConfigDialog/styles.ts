import * as colors from "material-ui/styles/colors"
const width = window.screen.width
const height = window.screen.height

export const styles = {
    parentStyle: {
        height: height + "px",
        width: width + "px"
    },
    flatButtonLabel: {
        color: colors.orange700
    },
    dialogTitle: {
        backgroundColor: colors.orange700,
        color: colors.white
    },
    gridStyle: {
        marginTop: 35
    },
    colStyle_margin: {
        marginTop: 20,
        display: "flex",
        justifyContent: "center" as "center"
    },
    colStyle: {
        marginTop: 20,
        display: "flex",
        justifyContent: "center" as "center"
    },
    blackIcon: {
        color: "black"
    },
    tab: {
        backgroundColor: colors.white,
        color: colors.orange700
    },
    floatingLabelFocus: {
        color: colors.orange700
    },
    underlineFolcusStyle: {
        borderColor: colors.orange700
    },
    contentStyle: {
        width: "30%"
    },
    textStyle: {
        width: "90%"
    }
}
