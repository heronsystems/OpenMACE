import * as materialColors from "material-ui/styles/colors"

export const styles = {
    parentContainer: {
        position: "absolute" as "absolute",
        top: 80,
        left: 26,
        display: "flex",
        justifyContent: "space-between" as "space-between",
        flexDirection: "row" as "row"
    },

    buttonsContainer: {
        display: "flex",
        justifyContent: "space-between" as "space-between",
        alignItems: "center" as "center",
        flexDirection: "column" as "column",
        zIndex: 9999
    },

    sliderContainer: {
        position: "relative" as "relative",
        left: 26,
        width: 250,
        zIndex: 100
    },

    inlineContainer: {
        display: "flex",
        justifyContent: "space-between" as "space-between",
        alignItems: "center" as "center",
        flexDirection: "row" as "row"
    },

    buttonStyle: {
        marginTop: 5,
        width: 120
    },

    textStyle: {
        color: materialColors.white,
        textAlign: "center" as "center",
        fontSize: 18
    },

    iconButtonStyle: {
        paddingBottom: 10,
        padding: 0,
        color: materialColors.white
    },

    sliderStyle: {
        padding: 0,
        marginBottom: 12,
        marginTop: 12
    }
}
