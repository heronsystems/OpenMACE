import colors from "../../../util/colors";

export default {
  heading: {
    fontWeight: 500,
    fontSize: 24,
    // color: colors.gray[900],
    color: colors.blue[100],
    marginTop: 0,
    marginBottom: 0
  },
  container: {
    margin: "0 24px"
  },
  shadowContainer: {
    padding: 16,
    boxShadow: "0px 2px 14px rgba(0,0,0,.08)",
    borderRadius: 8,
    backgroundColor: colors.white
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
  },
  header: {
    display: "flex",
    justifyContent: "space-between" as "space-between",
    alignItems: "center" as "center",
    marginBottom: 40
  },
  plotTypeSelect: {
    width: 200
  },
  countSelect: {
    marginRight: 24,
    width: 200
  },
  selectLabel: {
    fontSize: 12,
    marginBottom: 10,
    color: colors.blue[200],
    fontWeight: 500 as 500
  },
  selectLabelContainer: {
    marginBottom: 4
  },
  filters: {
    display: "flex"
  },
  utilityChartContainer: {
    display: "flex",
    flexDirection: "column" as "column"
  },
  segmentedControlContainer: {
    justifyContent: "center" as "center"
  }
};
