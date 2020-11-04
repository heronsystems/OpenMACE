import * as React from "react"
import styles from "./styles"

type Props = {
    radius: number
    position: { x: number; y: number }
    color?: string
    style?: React.CSSProperties
}

const Circle: React.StatelessComponent<Props> = (props: Props) => {
    const { style, radius, color } = props
    return (
        <div
            style={Object.assign({}, styles.circle, style, {
                width: radius * 2,
                height: radius * 2,
                borderRadius: radius * 2,
                backgroundColor: color
            })}
        ></div>
    )
}

Circle.defaultProps = {
    color: "red"
}

export default Circle
