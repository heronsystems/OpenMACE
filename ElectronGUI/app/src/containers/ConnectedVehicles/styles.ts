// const width = window.screen.width;
const height = window.screen.height

export const styles = {
    connectedVehiclesContainer: {
        position: "absolute" as "absolute",
        top: 64,
        right: 0,
        zIndex: 999,
        width: 20 + "%",
        backgroundColor: "rgba(255,255,255,1)",
        display: "flex",
        alignItems: "center" as "center",
        flexDirection: "column" as "column",
        maxHeight: height - 165,
        height: height,
        // overflowY: "scroll" as "scroll",
        overflowX: "hidden" as "hidden"
    },
    rangeSelectStyle: {
        position: "relative" as "relative",
        display: "flex",
        justifyContent: "center" as "center",
        alignItems: "center" as "center",
        zIndex: 9999
    }
}
