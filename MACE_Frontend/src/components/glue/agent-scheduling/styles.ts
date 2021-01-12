import colors from "../../../util/colors";

export default {
  heading: {
    fontWeight: 500,
    fontSize: 24,
    // color: colors.gray[900],
    color: colors.blue[100],
    marginTop: 0,
    display: "block"
  },
  outerContainer: {
    margin: 24,
    flex: 1
  },
  container: {
    backgroundColor: colors.white,
    boxShadow: "0px 2px 14px rgba(0,0,0,.08)",
    padding: 16,
    borderRadius: 8,
    flex: 1
  },
  noAgentsText: {
    color: colors.gray[600],
    fontSize: 14
  },
  tooltip: {
    padding: 16,
    borderRadius: 8,
    boxShadow: "0px 2px 14px rgba(0,0,0,.08)",
    display: "flex",
    flexDirection: "column" as "column",
    backgroundColor: colors.white
  },
  tooltipLabel: {
    color: colors.gray[700],
    fontSize: 14
  },
  tooltipValue: {
    color: colors.gray[900],
    fontSize: 14,
    marginLeft: 8
  },
  tooltipRow: {
    padding: "8px 0",
    display: "flex"
  }
};
