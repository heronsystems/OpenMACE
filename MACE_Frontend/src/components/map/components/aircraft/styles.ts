import colors from "../../../../util/colors";

export default {
  tooltip: {
    display: "flex",
    flexDirection: "column" as "column",
    padding: "4px 8px"
  },
  row: {
    padding: "2px 0"
  },
  label: {
    fontWeight: 500 as 500,
    fontSize: 13,
    color: colors.gray[800]
  },
  value: {
    fontSize: 13,
    color: colors.gray[700],
    marginLeft: 4
  },
  tailNumberContainer: {
    width: 24,
    height: 24,
    borderRadius: 50,
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    position: "absolute",
    left: 0,
    bottom: -16,
    borderWidth: 2,
    borderStyle: "solid",
    zIndex: 1000
  },
  tailNumberContainer_notification: {
    animation: "notificationAnimation 2s ease-in-out infinite",
    borderColor: colors.red[700],
    backgroundColor: colors.red[500]
  }
};
