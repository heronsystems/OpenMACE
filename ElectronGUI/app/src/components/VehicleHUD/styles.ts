import getMuiTheme from "material-ui/styles/getMuiTheme"
const lightMuiTheme = getMuiTheme()

export const styles = {
    avatar: {
        borderRadius: 30 + "%"
    },
    col_flat: {
        paddingTop: 0,
        paddingBottom: 0
    },
    menuStyle: {
        paddingLeft: 0,
        paddingRight: 0,
        paddingTop: -30,
        paddingBottom: 0,
        marginTop: -15,
        marginLeft: -30,
        marginRight: 0
    },
    menuItem: {
        color: "rgba(0,0,0,0.7)",
        width: -30,
        paddingLeft: 0,
        paddingRight: 0,
        paddingTop: 0,
        paddingBottom: 0,
        marginLeft: 0,
        marginRight: 0
    },
    iconStyle: {
        marginLeft: -50
    },
    col_thin: {
        paddingLeft: 0,
        paddingRight: 0
    },
    col_icon: {
        display: "flex" as "flex",
        justifyContent: "flex-end" as "flex-end",
        alignItems: "center" as "center",
        paddingLeft: 0,
        paddingRight: 0
    },
    col_iconLabel: {
        display: "flex" as "flex",
        justifyContent: "center" as "center",
        alignItems: "center" as "center",
        paddingLeft: 0,
        paddingRight: 0,
        paddingTop: 4 + "px"
    },
    iconLabelText: {
        fontSize: 12
    },
    cardHeaderText: {
        fontSize: 24
    },
    cardText: {
        fontSize: 18,
        paddingTop: 0,
        paddingBottom: 0
    },
    batteryContainer: {
        display: "flex" as "flex",
        justifyContent: "flex-end" as "flex-end",
        alignItems: "center" as "center",
        paddingLeft: 0,
        paddingRight: 0
    },
    batteryMenu: {
        padding: 0
    },
    batteryText: {
        fontSize: 18 + "px",
        display: "flex" as "flex",
        justifyContent: "flex-start" as "flex-start",
        alignItems: "center" as "center",
        paddingLeft: 0,
        paddingRight: 0,
        paddingTop: 11 + "px"
    },
    rpyText: {
        fontSize: 18,
        paddingTop: 0,
        paddingBottom: 0
    },
    messagesContainer: {
        width: "100%",
        height: "9px",
        borderBottom: "1px solid #8c8b8b",
        textAlign: "center" as "center"
    },
    messagesTitle: {
        backgroundColor: "#fff",
        padding: "0 10px"
    },
    // cardText_messages: {
    //     color: textSeverityToColor(this.props.aircraft.messages[0].severity),
    //     fontSize: 18,
    //     paddingTop: 0,
    //     paddingBottom: 0
    // },
    borderLine: {
        width: "100%",
        borderBottom: "1px solid #8c8b8b"
    },
    cardActions: {
        textAlign: "center" as "center"
    },
    selectField: {
        position: "relative" as "relative",
        right: 15,
        width: "140px",
        backgroundColor: lightMuiTheme.palette.canvasColor
    }
};
