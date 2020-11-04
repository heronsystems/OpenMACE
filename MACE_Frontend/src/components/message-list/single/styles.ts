import colors from "../../../util/colors";

export default {
  row: {
    display: "flex",
    flexDirection: "column" as "column",
    padding: 16,
    boxShadow: "0px 2px 14px rgba(0,0,0,.08)",
    backgroundColor: colors.white,
    borderRadius: 8,
    marginBottom: 8,
    zIndex: 999
  },
  rowTitle: {
    color: colors.gray[800]
  },
  date: {
    color: colors.gray[600],
    fontSize: 14
  },
  content: {
    display: "flex",
    alignItems: "center" as "center",
    justifyContent: "space-between" as "space-between"
  },
  expandText: {
    fontSize: 12,
    marginTop: 12,
    color: colors.gray[600],
    cursor: "pointer"
  },
  label: {
    display: "table-cell",
    fontSize: 13,
    paddingTop: 4,
    paddingBottom: 4,
    color: colors.gray[600]
  },
  value: {
    display: "table-cell",
    fontSize: 13,
    paddingLeft: 16,
    color: colors.gray[800],
    wordBreak: "break-all" as "break-all",
    lineHeight: 1.4
  },
  detailList: {
    listStyleType: "none",
    // padding: 0,
    columnCount: 1,
    backgroundColor: colors.gray[200],
    padding: "8px 16px",
    borderRadius: 8
  },
  detailRow: {
    // display: "table-row",
    padding: "7px 0"
  }
};
