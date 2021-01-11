import * as React from "react";
import { ResizableBox } from "react-resizable";
import {
  YAxis,
  XAxis,
  Line,
  LineChart,
  Legend,
  CartesianGrid,
  BarChart,
  Bar,
  Label
} from "recharts";
import "react-resizable/css/styles.css";
import styles from "./styles";
import colors from "../../util/colors";
import IndividualAgentUtility from "./individual-agent-utility";
import AgentScheduling from "./agent-scheduling";

// function getRandomColor() {
//     // var letters = "0123456789ABCDEF"
//     // var color = "#"
//     // for (var i = 0; i < 6; i++) {
//     //     color += letters[Math.floor(Math.random() * 16)]
//     // }
//     // return color
// }

/**
 *  For each vehicle/agent we will create a new bar chart instance
 *  Each bar chart instance may have multiple bars that stack
 *  There may be gaps in the bars that we shall render as transparent ?
 */

// const barData = [
//     { name: "taskA", start: 0, end: 150, duration: 150 },
//     { name: "taskB", start: 200, end: 250, duration: 50 }
// ]

const barData = [
  {
    name: "Agent A",
    taskA: { value: 235 },
    transit: { value: 50 },
    taskC: { value: 335 },
    taskF: { value: 453 }
  }
];

const data = [
  {
    time: 0,
    a: 4000,
    b: 2400,
    amt: 2400
  },
  {
    time: 10,
    a: 3000,
    b: 1398,
    amt: 2210
  },
  {
    time: 20,
    a: 2000,
    b: 9800,
    amt: 2290
  },
  {
    time: 30,
    a: 2780,
    b: 3908,
    amt: 2000
  },
  {
    time: 40,
    a: 1890,
    b: 4800,
    amt: 2181
  },
  {
    time: 50,
    a: 2390,
    b: 3800,
    amt: 2500
  },
  {
    time: 60,
    a: 3490,
    b: 4300,
    amt: 2100
  }
];

type Props = {};

export default (props: Props) => {
  return (
    <div style={styles.container}>
      <div style={styles.left}>
        <IndividualAgentUtility data={data} />
      </div>
      <div style={styles.right}>
        <AgentScheduling data={barData} />
      </div>
    </div>
  );
};
