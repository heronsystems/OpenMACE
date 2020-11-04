import colors from "../../../../util/colors";

export default {
  tooltip: {
    display: "flex",
    flexDirection: "column" as "column",
    padding: "4px 8px"
  },
  fixedTooltip: {
    display: "flex",
    flexDirection: "column" as "column",
    minWidth: 50
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
    position: "absolute" as "absolute",
    top: -24,
    left: 0,
    right: 0,
    margin: "0 auto",
    display: "flex",
    justifyContent: "center" as "center"
  },
  tailNumber: {
    fontSize: 16,
    color: colors.red[600],
    fontWeight: 500 as 500,
    textAlign: "center" as "center",
    textShadow: "0px 0px 12px rgba(0,0,0,.60)"
  }
};
