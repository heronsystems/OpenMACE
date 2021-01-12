import colors from "../../util/colors";

export default {
  absoluteContainer: {
    zIndex: 999,
    left: 0,
    bottom: 24,
    right: 0,
    margin: "0 auto",
    position: "absolute" as "absolute",
    width: "90%"
  },
  container: {
    borderRadius: 8,
    // backgroundColor: colors.offWhite,
    width: "100%",
    display: "flex",
    padding: 24,
    margin: "0px 24px"
  },
  innerContainer: {
    padding: 24,
    display: "flex"
  },
  handle: {
    width: 50,
    height: 5,
    backgroundColor: colors.gray[400],
    borderRadius: 10,
    top: 8,
    left: 0,
    right: 0,
    margin: "0 auto",
    cursor: "row-resize" as "row-resize",
    position: "absolute" as "absolute"
  },
  heading: {
    fontWeight: 500,
    fontSize: 24,
    color: colors.gray[900],
    marginTop: 0
  },
  left: {},
  right: {}
};
