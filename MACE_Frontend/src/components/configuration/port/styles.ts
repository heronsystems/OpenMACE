import colors from "../../../util/colors";

export default {
  container: {
    minWidth: "50%"
  },
  header: {
    paddingBottom: 16
  },
  heading: {
    fontSize: 20,
    fontWeight: 500 as 500,
    color: colors.gray[800]
  },
  buttonContainer: {
    display: "flex",
    justifyContent: "flex-end",
    marginTop: 16
  },
  saveButton: {
    padding: "10px 20px",
    border: "none",
    backgroundColor: colors.green[500],
    fontSize: 14,
    fontWeight: 500 as 500,
    color: colors.gray[900],
    borderRadius: 2,
    marginLeft: 10,
    cursor: "pointer" as "pointer",
    boxShadow: "0px 0px 15px rgba(0,0,0,.12)"
  },
  cancelButton: {
    padding: "10px 20px",
    backgroundColor: colors.gray[300],
    border: "none",
    // borderWidth: 1,
    // borderStyle: "solid",
    // borderColor: colors.gray[400],
    fontSize: 14,
    fontWeight: 500 as 500,
    color: colors.gray[900],
    borderRadius: 2,
    cursor: "pointer" as "pointer"
    // boxShadow: "0px 0px 15px rgba(0,0,0,.12)"
  },
  inputs: {},
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
  label: {
    fontSize: 12,
    // textTransform: "uppercase" as "uppercase",
    color: colors.gray[600]
  }
};
