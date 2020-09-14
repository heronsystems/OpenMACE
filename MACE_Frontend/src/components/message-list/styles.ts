import colors from "../../util/colors";

export default {
  container: {
    maxHeight: 250,
    overflowY: "scroll" as "scroll",
    zIndex: 989
  },
  outerContainer: {
    margin: 24,
    flex: 1,
    zIndex: 990
  },
  heading: {
    fontWeight: 500,
    fontSize: 24,
    // color: colors.gray[900],
    color: colors.blue[100],
    marginTop: 0
  },
  noMessagesText: {
    color: colors.gray[600],
    fontSize: 14
  },
  row: {
    display: "flex",
    flexDirection: "column" as "column",
    padding: 16,
    boxShadow: "0px 2px 14px rgba(0,0,0,.08)",
    backgroundColor: colors.white,
    borderRadius: 8,
    marginBottom: 8
  },
  noMessagesContainer: {
    display: "flex",
    flexDirection: "column" as "column",
    boxShadow: "0px 2px 14px rgba(0,0,0,.08)",
    backgroundColor: colors.white,
    borderRadius: 8,
    height: 51,
    justifyContent: "center" as "center",
    paddingLeft: 16,
    paddingRight: 16
  }
};
