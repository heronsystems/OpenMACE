import * as colors from "material-ui/styles/colors"

const width = window.screen.width
const height = window.screen.height

export const styles = {
    parentStyle: {
        height: height + "px",
        width: width + "px"
    },

    whiteButton: {
        color: "white"
    },

    appBar: {
        backgroundColor: colors.orange700,
        position: "fixed" as "fixed"
    }
}
