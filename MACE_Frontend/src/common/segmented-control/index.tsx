import * as React from "react";
import styles from "./styles";

type Option = {
  value: string;
  label: string;
};

type Options = Option[];

type Props = {
  options: Options;
  active?: string;
  onChange: (value: string) => void;
};

export default (props: Props) => {
  return (
    <div style={styles.container}>
      {props.options.map((o, index) => (
        <div
          key={index}
          onClick={() => props.onChange(o.value)}
          style={Object.assign(
            {},
            styles.option,
            index === 0 && styles.firstOption,
            index === props.options.length - 1 && styles.lastOption,
            props.active === o.value && styles.active
          )}
        >
          {o.label}
        </div>
      ))}
    </div>
  );
};
