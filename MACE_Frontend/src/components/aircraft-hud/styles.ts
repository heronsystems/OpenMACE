import colors from "../../util/colors";

export default {
  container: {
    position: "absolute" as "absolute",
    top: 24,
    right: 24,
    bottom: 24,
    zIndex: 999,
    // backgroundColor: "rgba(255,255,255,.6)",
    width: 300,
    overflowY: "scroll" as "scroll"
  },
  searchContainer: {
    marginBottom: 12,
    position: "relative" as "relative"
  },
  searchInput: {
    width: 300,
    height: 42,
    borderRadius: 4,
    border: "none",
    paddingLeft: 36,
    paddingRight: 8
  },
  searchIcon: {
    position: "absolute" as "absolute",
    left: 8,
    top: 0,
    bottom: 0,
    marginTop: "auto",
    marginBottom: "auto",
    color: colors.gray[700]
  },
  clearIcon: {
    position: "absolute" as "absolute",
    right: 8,
    top: 0,
    bottom: 0,
    marginTop: "auto",
    marginBottom: "auto",
    color: colors.gray[700],
    cursor: "pointer" as "pointer"
  },
  noResultsContainer: {
    width: 300,
    borderRadius: 4,
    justifyContent: "center" as "center",
    alignItems: "center" as "center",
    backgroundColor: colors.white,
    display: "flex",
    paddingTop: 16,
    paddingBottom: 16
  },
  noResultsText: {
    fontSize: 14,
    textAlign: "center" as "center",
    color: colors.gray[700]
  },
  commandsContainer: {
    marginBottom: 12,
    position: "relative" as "relative",
    backgroundColor: colors.gray[700],
    padding: 16,
    borderTopLeftRadius: 4,
    borderTopRightRadius: 4
  },
  commandButtons: {
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
    marginTop: 12
  },
  title: {
    fontSize: 16,
    fontWeight: 500 as 500,
    color: colors.blue[100]
  },
  centerButton: {
    backgroundColor: "transparent",
    border: "none",
    boxShadow: "none",
    display: "flex",
    cursor: "pointer"
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
  }
};
