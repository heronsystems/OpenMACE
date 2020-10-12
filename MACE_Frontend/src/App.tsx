import * as React from "react";
import { render } from "react-dom";
import "./styles.css";
import AppProvider from "./Provider";
import "react-notifications-component/dist/theme.css";
import "animate.css";
import { HashRouter, Route, Switch } from "react-router-dom";
import { Map as MapView } from "./routes";
import ReactNotification from 'react-notifications-component';
import 'react-notifications-component/dist/theme.css';


type Props = {};

type State = {};

class App extends React.Component<Props, State> {
  render() {
    return (
      <AppProvider>
        <div style={{ width: "100%", height: "100%" }}>
          <HashRouter basename="">
            <Switch>
              <Route path="/" exact component={MapView} />
            </Switch>
          </HashRouter>
        </div>

        <ReactNotification />
      </AppProvider>
    );
  }
}

render(<App />, document.getElementById("app"));
