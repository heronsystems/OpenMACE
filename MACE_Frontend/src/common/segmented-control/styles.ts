import colors from "../../util/colors";

export default {
  container: {
    display: "flex",
    justifyContent: "center" as "center"
  },
  option: {
    fontSize: 13,
    borderWidth: 1,
    borderLeftWidth: 0,
    borderStyle: "solid",
    borderColor: colors.gray[400],
    padding: "4px 8px",
    cursor: "pointer"
  },
  firstOption: {
    borderTopLeftRadius: 4,
    borderBottomLeftRadius: 4,
    borderLeftWidth: 1
  },
  lastOption: {
    borderTopRightRadius: 4,
    borderBottomRightRadius: 4
  },
  active: {
    backgroundColor: colors.gray[400]
    // borderColor: colors.green[400]
  }
};
