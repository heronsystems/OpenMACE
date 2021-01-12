import * as React from "react";
import styles from "./styles";
import colors from "../../../util/colors";
import {
  Line,
  LineChart,
  XAxis,
  YAxis,
  Legend,
  ResponsiveContainer,
  Text
} from "recharts";
import {
  formatIndividualAgentUtility,
  getColorForIndex,
  formatTotalUtility
} from "../../../util/helpers";
import NoData from "../no-data";
import moment from "moment";
import Select from "react-select";
import SegmentedControl from "../../../common/segmented-control";

type Props = {
  utility: { [agentId: string]: { time: number; value: number }[] };
  totalUtility: { time: number; value: number }[];
};

const PLOT_TYPE_OPTIONS = [
  { value: "utility", label: "Utility" }
];

const CUMULATIVE_OPTIONS = [
  { value: "non-cumulative", label: "Per-Agent" },
  { value: "cumulative", label: "Total" }
];

export default (props: Props) => {
  const [plotType, setPlotType] = React.useState(PLOT_TYPE_OPTIONS[0]);
  const [cumulative, setCumulative] = React.useState("cumulative");
  const { utility, totalUtility } = props;
  const getDataForState = () => {
    if (plotType === PLOT_TYPE_OPTIONS[0]) {
      if (cumulative === "cumulative") {
        return formatTotalUtility(totalUtility);
      } else if (cumulative === "non-cumulative") {
        return formatIndividualAgentUtility(utility);
      }
    }
  };
  const getAgents = () => {
    if (plotType === PLOT_TYPE_OPTIONS[0]) {
      return Object.keys(utility);
    }
  };
  const agents = getAgents();
  const formatTime = (time: number) => {
    if (typeof time === "number") {
      // TODO PAT: May have to change this to nano seconds
      // const m = moment(time / 1000000); // Convert nanoseconds to msecs
      // return m.format("HH:mm:ss");
      return moment(time / 1000000).format("HH:mm:ss");
    }
    return "";
  };
  const CustomYAxisLabel = ({ label }: { label: string }) => {
    return (
      <Text
        x={-30}
        y={50}
        dx={50}
        dy={150}
        angle={-90}
        fontSize="12"
        fill={colors.gray[700]}
        fontWeight={400}
      >
        {label}
      </Text>
    );
  };
  const getTitleBasedOnFilters = () => {
    if (cumulative === "cumulative" && plotType.value === "utility") {
      return "Total Utility";
    } else if (
      cumulative === "non-cumulative" &&
      plotType.value === "utility"
    ) {
      return "Individual Agent Utility";
    }
  };
  const formatLegendContent = (value) => {
    if (cumulative === "cumulative") {
      return "Total";
    }
    return value;
  };
  const getLabelForPlotType = () => {
    if (plotType === PLOT_TYPE_OPTIONS[0]) {
      return "Utility";
    }
  };
  const getTicks = () => {
    const dataWithoutFiller = data.filter((d) => d.time);
    const first = dataWithoutFiller[0];
    const last = dataWithoutFiller[dataWithoutFiller.length - 1];
    let middle = { time: "" };
    let second = { time: "" };
    let fourth = { time: "" };
    const ticks = [""];
    if (first) {
      ticks.pop();
      ticks.push(first.time);
    }
    if (dataWithoutFiller.length > 5) {
      ticks.push(last.time);
    }
    if (dataWithoutFiller.length >= 12) {
      const middleIndex = Math.round(dataWithoutFiller.length / 2);
      middle = data[middleIndex - 1];
      ticks.splice(1, 0, middle.time);
      if (dataWithoutFiller.length >= 20) {
        const secondIndex = Math.round((middleIndex - 0) / 2);
        second = data[secondIndex - 1];
        ticks.splice(1, 0, second.time);
        const fourthIndex =
          dataWithoutFiller.length -
          Math.round((dataWithoutFiller.length - middleIndex) / 2);
        fourth = data[fourthIndex - 1];
        ticks.splice(3, 0, fourth.time);
      }
    }
    return ticks;
  };
  const checkIfRealDataExists = (data) => {
    const found = data.findIndex((d) => d.time !== null && d.value !== null);
    if (found !== -1) {
      return true;
    }
    return false;
  };
  const data = getDataForState();
  const realDataExists = checkIfRealDataExists(data);
  const title = getTitleBasedOnFilters();
  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <p style={styles.heading}>{title}</p>
        <div style={styles.filters}>
          <div style={styles.plotTypeSelect}>
            <div style={styles.selectLabelContainer}>
              <span style={styles.selectLabel}>Plot Type</span>
            </div>
            <Select
              options={PLOT_TYPE_OPTIONS}
              value={plotType}
              onChange={(ev) => {
                setPlotType(ev);
              }}
            />
          </div>
        </div>
      </div>
      <div style={styles.shadowContainer}>
        {realDataExists ? (
          <div style={styles.utilityChartContainer}>
            <div style={styles.segmentedControlContainer}>
              <SegmentedControl
                options={CUMULATIVE_OPTIONS}
                onChange={setCumulative}
                active={cumulative}
              />
            </div>
            <ResponsiveContainer height={400}>
              <LineChart data={data} margin={{ left: 0, right: 24, top: 20 }}>
                <XAxis
                  dataKey="time"
                  tick={{ fontSize: 12 }}
                  tickFormatter={formatTime}
                  interval={0}
                  tickCount={5}
                  allowDataOverflow
                  ticks={getTicks()}
                />
                <YAxis
                  tick={{ fontSize: 12 }}
                  label={
                    (<CustomYAxisLabel label={getLabelForPlotType()} />) as any
                  }
                />

                {cumulative === "non-cumulative" ? (
                  agents.map((a, index) => (
                    <Line
                      dot={false}
                      key={a}
                      type="linear"
                      dataKey={a}
                      stroke={getColorForIndex(index)}
                      strokeWidth={2}
                      connectNulls
                      isAnimationActive={false}
                    />
                  ))
                ) : (
                  <Line
                    dataKey={"value"}
                    stroke={colors.green[500]}
                    strokeWidth={2}
                    isAnimationActive={false}
                    type="linear"
                    dot={false}
                  />
                )}
                <Legend
                  height={24}
                  iconType="rect"
                  iconSize={12}
                  formatter={formatLegendContent}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        ) : (
          <NoData
            onChangeCumulative={setCumulative}
            cumulative={cumulative}
            options={CUMULATIVE_OPTIONS}
          />
        )}
      </div>
    </div>
  );
};
