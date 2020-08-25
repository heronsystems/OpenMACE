import * as React from "react"
import { FiAlertCircle } from "react-icons/fi"
import styles from "./styles"

type Props = {}

export default (props: Props) => {
    return (
        <div style={styles.container}>
            <FiAlertCircle style={styles.icon} />
            <span style={styles.message}>This is a test message</span>
        </div>
    )
}
