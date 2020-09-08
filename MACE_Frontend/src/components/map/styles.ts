import colors from "../../util/colors";
export default {
  centerButton: {
    backgroundColor: "transparent",
    border: "none",
    boxShadow: "none",
    display: "flex",
    cursor: "pointer",
    padding: "4px 8px"
  },
  container: {
    zIndex: 9999,
    padding: 12,
    backgroundColor: colors.white,
    borderRadius: 4,
    boxShadow: "0px 0px 15px rgba(0,0,0,.25)"
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
  label: {
    fontSize: 14,
    color: colors.gray[700],
    display: "table-cell",
    padding: "4px 8px 4px 0"
  },
  option: {
    cursor: "pointer",
    fontSize: 13,
  },
  row: {
    padding: "4px 16px",
    display: "flex",
    justifyContent: "space-between"
  },
  selectRow: {
    width: "200px",
    fontSize: 18
  },
  singleSettingContainer: {
    display: "flex",
    flexDirection: "column" as "column",
    marginBottom: 16
  },
  tooltipButtons: {
    display: "flex",
    alignItems: "left"
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
  }
};