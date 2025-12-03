import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import PropTypes from "prop-types";

class Unknown extends Component {
  renderLogger = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, cell } = this.props;
    const prefixCls = getPrefixCls("unknown-cell-type", customizePrefixCls);
    return (
      <div className={`${prefixCls}`}>
        <div>不支持的组件类 {cell.type}</div>
      </div>
    );
  };

  render() {
    return <ConfigConsumer>{this.renderLogger}</ConfigConsumer>;
  }
}

Unknown.propTypes = {
  cell: PropTypes.object.isRequired,
};

export default Unknown;
