import colors from "../../util/colors";

export default {

    centerButton: {
        backgroundColor: "transparent",
        border: "none",
        // boxShadow: "none",
        display: "flex",
        cursor: "pointer",
    },
    container: {
        position: "absolute" as "absolute",
        top: 14,
        left: 14,
        zIndex: 999,
        display: "flex"
    },
    buttonContainer: {
        position: "relative" as "relative",
        display: "flex",
    },
    testButtonContainer: {
        position: "relative" as "relative",
        top: 0,
        left: 14,
        display: "flex",
    },

    singleSettingContainer: {
        display: "flex",
        flexDirection: "column" as "column",
        marginBottom: 16
    },

    inputLabel: {
        fontSize: 12,
        color: colors.gray[600]
    },

  input: {
    borderWidth: 1,
    borderStyle: "solid" as "solid",
    borderColor: colors.gray[400],
    padding: "8px 4px",
    borderRadius: 2,
    backgroundColor: colors.gray[100],
    marginTop: 4
  },

  tooltipButtons: {
    display: "flex",
    alignItems: "left"
  },

  tooltipContainer_rel: {
      position: "relative" as "relative",
      top: 24,
      left: 54
  },
  tooltipContainer_abs: {
    position: "absolute" as "absolute",
    top: 0,
    left: 34,
    backgroundColor: colors.white,
    boxShadow: "0px 2px 30px rgba(0,0,0,.25)",
    borderBottomLeftRadius: 8,
    borderBottomRightRadius: 8,
    borderTopLeftRadius: 4,
    borderTopRightRadius: 4
  },
  tooltipContentContainer: {
      padding: 8
  }
};
