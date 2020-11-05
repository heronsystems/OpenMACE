import colors from "../../../util/colors";

export default {
  container: {
    backgroundColor: colors.white,
    boxShadow: "0px 2px 30px rgba(0,0,0,.25)",
    borderRadius: 8,
    marginBottom: 8
  },
  header: {
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
    // backgroundColor: colors.gray[200],
    backgroundColor: colors.teal[500],
    padding: 16,
    borderTopLeftRadius: 4,
    borderTopRightRadius: 4
  },
  evenRow: {
    backgroundColor: colors.gray[100]
  },
  last: {
    borderBottomLeftRadius: 4,
    borderBottomRightRadius: 4
  },
  centerButton: {
    backgroundColor: "transparent",
    border: "none",
    boxShadow: "none",
    display: "flex",
    cursor: "pointer"
  },
  disabled_centerButton: {
    backgroundColor: "transparent",
    border: "none",
    boxShadow: "none",
    display: "flex",
    cursor: "not-allowed"
  },
  title: {
    fontSize: 20,
    fontWeight: 500 as 500,
    color: colors.teal[100]
    // color: colors.gray[700]
  },
  row: {
    padding: "4px 16px",
    display: "flex",
    justifyContent: "space-between"
  },
  selectRow: {
    padding: "4px 16px",
  },
  hudRow: {
    padding: "4px 16px",
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
    width: 100+"%"
  },
  label: {
    fontSize: 14,
    color: colors.gray[700],
    display: "table-cell",
    padding: "4px 8px 4px 0"
  },
  value: {
    fontSize: 14,
    color: colors.black,
    display: "table-cell",
    padding: "4px 0",
    fontVariantNumeric: "tabular-nums"
  },
  commandsContainer: {
    position: "relative" as "relative",
    backgroundColor: colors.white,
    padding: 16,
    borderTopLeftRadius: 4,
    borderTopRightRadius: 4
  },
  commandButtons: {
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between"
  },
  commandButtonsNoGoHere: {
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between"
  },
  rotatedIcon: {
    transform: `rotate(-90deg)`
  },
  hudData: {
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between"
  },
  hudElement: {
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between"
  },
  hudValue: {
    fontSize: 14,
    color: colors.black,
    display: "table-cell",
    padding: "4px 4px",
    fontVariantNumeric: "tabular-nums"
  },
  indicatorContainer: {
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
    height: 150,
    pointerEvents: "none" as "none"
  },
  indicator: {
      width: 100+"px",
      height: 100+"px"
  },
  textContainer: {
    marginBottom: 12,
    position: "relative" as "relative",
    backgroundColor: colors.white,
    padding: 16,
    borderTopLeftRadius: 4,
    borderTopRightRadius: 4
  },
  singleSettingContainer: {
    display: "flex",
    flexDirection: "column" as "column",
    marginBottom: 16
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
  inputLabel: {
    fontSize: 12,
    // textTransform: "uppercase" as "uppercase",
    color: colors.gray[600]
  },
  tooltipButtons: {
    display: "flex",
    alignItems: "left"
  },
  altitudeTooltip: {
      backgroundColor: colors.black,
      position: "absolute" as "absolute",
      right: 200,
      zIndex: 9999
  },
  segmentedControlContainer: {
    justifyContent: "center" as "center"
  }
};
