import Colors from "../../util/colors";
import Fonts from "../../util/fonts";
import colors from "../../util/colors";

export default {
  container: {
    backgroundColor: "rgba(255, 255, 255, .9)",
    borderRadius: 4,
    padding: "8px 16px",
    position: "absolute" as "absolute",
    top: 24,
    right: 24,
    zIndex: 999,
    boxShadow: "0px 2px 4px rgba(0,0,0,.12)",
    display: "flex",
    alignItems: "center",
    flexDirection: "column" as "column"
  },
  message: {
    fontSize: 14,
    fontFamily: Fonts.heading,
    marginLeft: 16,
    fontWeight: 400 as 400,
    color: Colors.gray[700]
  },
  icon: {
    color: Colors.red[500],
    fontSize: 24
  },
  ctaContainer: {
    marginTop: 8,
    display: "flex",
    justifyContent: "flex-start",
    paddingBottom: 4
  },
  content: {
    display: "flex",
    alignItems: "center"
  },
  button: {
    backgroundColor: colors.gray[400],
    border: "none",
    color: colors.gray[700],
    padding: "10px 16px",
    borderRadius: 3,
    cursor: "pointer" as "pointer"
  }
};
