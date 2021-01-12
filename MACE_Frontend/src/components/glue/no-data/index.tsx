import * as React from "react";
import { Area, AreaChart, ResponsiveContainer } from "recharts";
import styles from "./styles";
import colors from "../../../util/colors";
import SegmentedControl from "../../../common/segmented-control";

const data = [
  { fake: 200 },
  { fake: 210 },
  { fake: 220 },
  { fake: 190 },
  { fake: 220 },
  { fake: 230 },
  { fake: 170 },
  { fake: 210 },
  { fake: 250 }
];
type Props = {
  options: { label: string; value: string }[];
  onChangeCumulative: (value: string) => void;
  cumulative: string;
};

export default (props: Props) => (
  <div style={styles.noDataContainer} data-testid="no-data">
    <SegmentedControl
      options={props.options}
      onChange={props.onChangeCumulative}
      active={props.cumulative}
    />
    <ResponsiveContainer height={400}>
      <AreaChart data={data}>
        <Area
          type="natural"
          dataKey="fake"
          stroke="transparent"
          fill={colors.gray[300]}
        />
      </AreaChart>
    </ResponsiveContainer>
    <p style={styles.noDataText}>No Data</p>
  </div>
);
