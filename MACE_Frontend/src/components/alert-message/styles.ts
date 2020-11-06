import Colors from "../../util/colors"
import Fonts from "../../util/fonts"

export default {
    container: {
        backgroundColor: "rgba(255, 255, 255, .9)",
        borderRadius: 4,
        padding: "8px 16px",
        position: "absolute" as "absolute",
        top: 24,
        right: 24,
        zIndex: 999,
        boxShadow: "0px 2px 4px rgba(0,0,0,.12)",
        display: "flex",
        alignItems: "center"
    },
    message: {
        fontSize: 14,
        fontFamily: Fonts.heading,
        marginLeft: 16,
        fontWeight: 400 as 400,
        color: Colors.gray[700]
    },
    icon: {
        color: Colors.red[500],
        fontSize: 18
    }
}
