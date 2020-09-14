import * as React from "react";
const { useContext } = React;
import AppContext, { Context } from "../../Context";
import styles from "./styles";
import Message from "./single";
import { CSSTransition, TransitionGroup } from "react-transition-group";

export default () => {
  const { messages } = useContext<Context>(AppContext);
  return (
    <div style={styles.outerContainer}>
      <p style={styles.heading}>Message List</p>
      <div style={styles.container}>
        {messages.length ? (
          <TransitionGroup className="items-section__list">
            {messages
              .filter((m) => {
                if (m.hasOwnProperty("should_display")) {
                  if (m.should_display) {
                    return m;
                  }
                } else {
                  return m;
                }
              })
              .sort((a, b) => b.date - a.date)
              .map((m, i) => (
                <CSSTransition key={m.date} timeout={500} classNames="move">
                  <Message data={m} key={m.date} />
                </CSSTransition>
              ))}
          </TransitionGroup>
        ) : (
          <div style={styles.noMessagesContainer}>
            <span style={styles.noMessagesText}>No Messages</span>
          </div>
        )}
      </div>
    </div>
  );
};
