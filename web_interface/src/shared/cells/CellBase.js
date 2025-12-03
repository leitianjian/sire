import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
class CellBase extends Component {
  getOptions = () => {
    return !this.props.cell.options ? {} : this.props.cell.options;
  };

  updateOptions = (options) => {
    const cell = { ...this.props.cell };
    cell.options = { ...cell.options, ...options };
    this.props.onCellUpdate(cell);
  };

  renderContent = ({ prefixCls }) => {
    throw new Error("renderComponent not implemented");
  };

  render() {
    return <ConfigConsumer>{this.renderContent}</ConfigConsumer>;
  }
}

export default CellBase;
