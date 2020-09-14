import colors from "../../../../../util/colors";

export default {
  manualEntryContainer: {
    paddingTop: 8,
    paddingBottom: 8
  },
  actionsContainer: {
    paddingTop: 8,
    paddingBottom: 8
  },
  saveButton: {
    backgroundColor: colors.green[600],
    border: 0,
    color: colors.green[100],
    borderRadius: 2,
    fontWeight: 500 as 500,
    fontSize: 11,
    cursor: "pointer",
    marginRight: 4,
    padding: "4px 8px"
  },
  cancelButton: {
    backgroundColor: colors.gray[400],
    color: colors.gray[700],
    border: 0,
    borderRadius: 2,
    fontWeight: 500 as 500,
    fontSize: 11,
    cursor: "pointer",
    padding: "4px 8px"
  },
  removeButton: {
    backgroundColor: colors.red[600],
    color: colors.red[100],
    border: 0,
    borderRadius: 2,
    fontWeight: 500 as 500,
    fontSize: 11,
    cursor: "pointer",
    padding: "4px 8px",
    marginLeft: 4
  },
  transparentButton: {
    background: "transparent",
    border: 0,
    boxShadow: "none",
    padding: 0,
    cursor: "pointer",
    fontSize: 10,
    color: colors.blue[800]
  },
  inputContainer: {
    display: "flex",
    flexDirection: "column" as "column"
  },
  input: {
    paddingLeft: 4,
    paddingTop: 4,
    paddingBottom: 4,
    borderRadius: 2,
    borderColor: colors.gray[300],
    borderWidth: 1,
    boxShadow: "none",
    borderStyle: "solid"
  }
};
