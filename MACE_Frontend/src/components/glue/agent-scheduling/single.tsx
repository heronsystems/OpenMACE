import * as React from "react";
import {
  BarChart,
  XAxis,
  YAxis,
  Bar,
  Tooltip,
  ResponsiveContainer
} from "recharts";
import styles from "./styles";
import colors from "../../../util/colors";
import {
  formatAgentScheduling,
  getColorForIndex,
  fmtMSS,
  sToHMS
} from "../../../util/helpers";
const transitRegex = new RegExp(/transit/);

type Props = {
  data: GLUE.SchedulePayload;
  index: number;
};

export default (props: Props) => {
  let active = null;
  const { data } = props;
  const formatData = () => {
    const { taskSchedule, agentID } = data;
    const formattedTasks = formatAgentScheduling(taskSchedule);
    return [formattedTasks];
  };

  const formatted = formatData();
  const _renderTooltip = ({ active: isActive, payload }) => {
    if (isActive && active) {
      const activeTask = payload.find(p => {
        return p.name === active;
      });
      if (transitRegex.test(active)) {
        return null;
      }
    //   console.log(Math.round(activeTask.value));
      return (
        <div style={styles.tooltip}>
          <div style={styles.tooltipRow}>
            <span style={styles.tooltipLabel}>Assignment:</span>{" "}
            <span style={styles.tooltipValue}>{activeTask.id}</span>
          </div>
          <div style={styles.tooltipRow}>
            <span style={styles.tooltipLabel}>Time:</span>{" "}
            <span style={styles.tooltipValue}>
              {sToHMS(Math.round(activeTask.value))}
            </span>
          </div>
        </div>
      );
    }
    return null;
  };
  return (
    <ResponsiveContainer height={90}>
      <BarChart
        layout="vertical"
        data={formatted}
        margin={{
          top: 15,
          right: 30,
          left: 0,
          bottom: 15
        }}
      >
        <XAxis
          type="number"
          tick={{ fontSize: 12 }}
          label={{
            value: "Time (sec)",
            position: "bottom",
            offset: 0,
            fontSize: 12,
            fontWeight: 400,
            fill: colors.gray[700]
          }}
        />
        <YAxis
          type="category"
          dataKey="name"
          tick={false}
          label={{
            value: data.agentID,
            style: { marginRight: 8 },
            fontSize: 12,
            fontWeight: 400,
            fill: colors.gray[700]
          }}
        ></YAxis>
        <Tooltip cursor={false} content={_renderTooltip} />
        {Object.keys(formatted[0]).map((t, index) => {
          const regex = new RegExp(/transit/g);
          const transit = regex.test(t);
          const colorIndex = (props.index + 1) * (index + 1);
          return (
            <Bar
              dataKey={`${t}.value`}
              fill={transit ? colors.transparent : getColorForIndex(colorIndex)}
              stackId="a"
              key={t}
              onMouseEnter={() => (active = `${t}.value`)}
              onMouseLeave={() => (active = null)}
              id={t}
            />
          );
        })}
      </BarChart>
    </ResponsiveContainer>
  );
};
