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
  }
};
