import React, { Component } from "react";
import { connect } from "react-redux";
import PropTypes from "prop-types";
import routes from "../routes";
import Header from "./Header";
import { getDashboards } from "../state/selectors";
import {
  loadDashboards,
  navigate,
  resetErrorMessage,
  updateRouterState,
} from "../actions";
import { ConfigConsumer } from "antd/lib/config-provider";

class App extends Component {
  constructor(props) {
    super(props);
    this.handleChange = this.handleChange.bind(this);
    this.handleDismissClick = this.handleDismissClick.bind(this);
  }

  componentDidMount() {
    this.props.loadConfig();
    this.props.updateRouterState({
      pathname: this.props.location.pathname,
      params: this.props.params,
    });
  }

  componentWillReceiveProps(nextProps) {
    if (this.props.location.pathname !== nextProps.location.pathname) {
      this.props.updateRouterState({
        pathname: nextProps.location.pathname,
        params: nextProps.params,
      });
    }
  }

  handleDismissClick(e) {
    this.props.resetErrorMessage();
    e.preventDefault();
  }

  handleChange(nextValue) {
    this.props.navigate(`/${nextValue}`);
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
    const {
      location: { pathname },
      dashboards,
      navigate,
      prefixCls: customizePrefixCls,
    } = this.props;
    const prefixCls = getPrefixCls("app", customizePrefixCls);
    return (
      <div className={prefixCls}>
        <Header
          className={`${prefixCls}-header`}
          pathname={pathname}
          dashboards={dashboards}
          navigate={navigate}
        />
        <div className={`${prefixCls}-content`}>
          {this.renderErrorMessage()}
          {routes}
        </div>
      </div>
    );
  };

  render() {
    return <ConfigConsumer>{this.renderApp}</ConfigConsumer>;
  }
}

App.propTypes = {
  // Injected by React Redux
  errorMessage: PropTypes.string,
  inputValue: PropTypes.string.isRequired,
  navigate: PropTypes.func.isRequired,
  updateRouterState: PropTypes.func.isRequired,
  resetErrorMessage: PropTypes.func.isRequired,
  loadConfig: PropTypes.func.isRequired,

  // Injected by React Router
  children: PropTypes.node,
  location: PropTypes.any,
  params: PropTypes.any,
};

function mapStateToProps(state) {
  return {
    dashboards: getDashboards(state),
    errorMessage: state.errorMessage,
    inputValue: state.router.pathname.substring(1),
  };
}

export default connect(mapStateToProps, {
  loadConfig: loadDashboards,
  navigate,
  updateRouterState,
  resetErrorMessage,
})(App);
