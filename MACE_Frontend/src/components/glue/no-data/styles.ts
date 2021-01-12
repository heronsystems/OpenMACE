import colors from "../../../util/colors";

export default {
  noDataContainer: {
    display: "flex",
    alignItems: "center" as "center",
    justifyContent: "center" as "center",
    flexDirection: "column" as "column"
  },
  noDataText: {
    color: colors.gray[500],
    fontWeight: 600 as 600,
    fontSize: 15,
    textTransform: "uppercase" as "uppercase",
    position: "absolute" as "absolute"
  }
};
