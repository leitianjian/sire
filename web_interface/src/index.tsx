import React from "react";
import { createRoot } from "react-dom/client";
import * as serviceWorker from "./serviceWorker";

import { Router } from "react-router-dom";
import { Provider } from "react-redux";
import App from "./containers/App";
import configureAppStore from "./store";
import rootSaga from "./state/sagas";
import { history } from "./state/api";
import ConfigProvider from "antd/lib/config-provider";
import "./style/index.less";

const store = configureAppStore();
store.runSaga(rootSaga);

export default store;
const root = createRoot(document.getElementById("root") as HTMLElement);
root.render(
  <Provider store={store}>
    <ConfigProvider prefixCls={"rchmi"}>
      <Router location={history.location} navigator={history}>
        <App />
      </Router>
    </ConfigProvider>
  </Provider>
);

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
