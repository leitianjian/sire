import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import PropTypes from "prop-types";

class Logger extends Component {
  renderLogger = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;
    const prefixCls = getPrefixCls("logger", customizePrefixCls);
    return <div className={`${prefixCls}`}></div>;
  };

  render() {
    return <ConfigConsumer>{this.renderLogger}</ConfigConsumer>;
  }
}

Logger.propTypes = {
  cell: PropTypes.object.isRequired,
};

Logger.NAME = "日志回放";

export default Logger;
