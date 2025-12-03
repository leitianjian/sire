import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import PropTypes from "prop-types";

class ConfigBase extends Component {
  getOptions = () => {
    const { cell } = this.props;
    if (!cell.options) {
      return {};
    } else {
      return cell.options;
    }
  };

  updateOptions = (options) => {
    const cell = { ...this.props.cell };
    cell.options = { ...cell.options, ...options };
    this.props.onCellUpdate(cell);
  };

  showConfig = () => {
    const options = this.getOptions();
    this.updateOptions({
      ...options,
      showConfig: true,
    });
  };

  hideConfig = () => {
    const options = this.getOptions();
    this.updateOptions({
      ...options,
      showConfig: null,
    });
  };

  renderContent = ({ prefixCls }) => {
    throw new Error("renderContent not implemented");
  };

  render() {
    return <ConfigConsumer>{this.renderContent}</ConfigConsumer>;
  }
}

ConfigBase.propTypes = {
  cell: PropTypes.object.isRequired,
  onCellUpdate: PropTypes.func.isRequired,
};

export default ConfigBase;
