import React, { Component } from "react";
import { connect } from "react-redux";
import routes from "../routes";
import Header from "./Header";
import Footer from "./Footer";
import { LoadingOutlined } from "@ant-design/icons";
import {
  getDashboards,
  getCurrentToolId,
  getCurrentWobjId,
  getGetInterval,
} from "../state/selectors";
import {
  loadInterfaceConfig,
  resetErrorMessage,
  updateRouterState,
  updateServerStateRequest,
} from "../state/actions";
import { ConfigConsumer } from "antd/lib/config-provider";

class App extends Component {
  constructor(props) {
    super(props);
    this.handleDismissClick = this.handleDismissClick.bind(this);
  }

  componentDidMount() {
    this.props.loadInterfaceConfig();
    this.props.updateRouterState({
      pathname: this.props.pathname,
      params: this.props.params,
    });
    this.intervalTimer = setInterval(() => {
      this.props.updateServerStateRequest({
        toolId: this.props.toolId,
        wobjId: this.props.wobjId,
      });
    }, 200);
  }

  componentWillUnmount() {
    clearInterval(this.intervalTimer);
  }

  componentWillReceiveProps(nextProps, nextContext) {
    if (this.props.pathname !== nextProps.pathname) {
      this.props.updateRouterState({
        pathname: nextProps.pathname,
        params: nextProps.params,
      });
    }
  }

  handleDismissClick(e) {
    this.props.resetErrorMessage();
    e.preventDefault();
  }

  renderErrorMessage() {
    const { errorMessage } = this.props;
    if (!errorMessage) {
      return null;
    }

    return (
      <p style={{ backgroundColor: "#e99", padding: 10 }}>
        <b>{errorMessage}</b> (
        <a href="#" onClick={this.handleDismissClick}>
          Dismiss
        </a>
        )
      </p>
    );
  }

  renderApp = ({ getPrefixCls }) => {
    const { dashboards, prefixCls: customizePrefixCls } = this.props;
    const prefixCls = getPrefixCls("app", customizePrefixCls);
    if (!dashboards) {
      return (
        <div className={`${prefixCls}-loading`}>
          <LoadingOutlined />
        </div>
      );
    }

    return (
      <div className={prefixCls}>
        <Header
          menus={dashboards.map((d) => ({
            key: `/dashboards/${d.i}`,
            name: d.name,
          }))}
        />
        <div className={`${prefixCls}-content`}>
          {this.renderErrorMessage()}
          {routes}
        </div>
        <Footer />
      </div>
    );
  };

  render() {
    return <ConfigConsumer>{this.renderApp}</ConfigConsumer>;
  }
}

function mapStateToProps(state) {
  return {
    dashboards: getDashboards(state),
    errorMessage: state.errorMessage,
    websocketConnected: state.robot.websocketConnected,
    getInterval: getGetInterval(state),
    toolId: getCurrentToolId(state),
    wobjId: getCurrentWobjId(state),
    pathname: state.route && state.route.pathname,
  };
}

function mapDispatchToProps(dispatch) {
  return {
    loadInterfaceConfig: (...args) => dispatch(loadInterfaceConfig(...args)),
    updateRouterState: (...args) => dispatch(updateRouterState(...args)),
    resetErrorMessage: (...args) => dispatch(resetErrorMessage(...args)),
    updateServerStateRequest: (...args) =>
      dispatch(updateServerStateRequest(...args)),
  };
}

export default connect(mapStateToProps, mapDispatchToProps)(App);
