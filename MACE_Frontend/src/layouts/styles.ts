import colors from "../util/colors";

export default {
  version: {
    position: "absolute" as "absolute",
    left: 0,
    bottom: 0,
    backgroundColor: "rgba(255,255,255, .5)",
    zIndex: 999,
    paddingLeft: 8,
    paddingTop: 2,
    paddingBottom: 2,
    paddingRight: 8
  },
  versionText: {
    fontSize: 11,
    color: colors.gray[800]
  },
  versionNumber: {
    fontWeight: 500 as 500
  }
};
