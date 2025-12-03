import React, { Component, Fragment } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import classnames from "classnames";
import PropTypes from "prop-types";

class LayoutCellHeader extends Component {
  renderLayoutCellHeader = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, isEditable } = this.props;
    const prefixCls = getPrefixCls("layout-cell-header", customizePrefixCls);
    const cellPrefixCls = getPrefixCls("layout-cell", customizePrefixCls);
    const draggable = `${cellPrefixCls}--draggable`;
    const headerDraggable = `${prefixCls}--draggable`;
    const extraClassname = {};
    extraClassname[draggable] = isEditable;
    extraClassname[headerDraggable] = isEditable;
    return (
      <div className={classnames(`${prefixCls}`, { ...extraClassname })}>
        {this.cellName(prefixCls)}
        {this.headingBar(prefixCls)}
      </div>
    );
  };

  cellName = (prefixCls) => {
    const { cellName } = this.props;
    const isDefault = `${prefixCls}-name__default`;
    const extraClassname = {};
    extraClassname[isDefault] = !cellName || cellName.trim() === "";
    const className = classnames(`${prefixCls}-name`, { ...extraClassname });
    return <span className={className}>{cellName || "未命名"}</span>;
  };

  headingBar = (prefixCls) => {
    const { isEditable } = this.props;
    if (isEditable) {
      return (
        <Fragment>
          <div className={`${prefixCls}--header-bar`} />
          <div className={`${prefixCls}--header-dragger`} />
        </Fragment>
      );
    }
  };

  render() {
    return <ConfigConsumer>{this.renderLayoutCellHeader}</ConfigConsumer>;
  }
}

LayoutCellHeader.propTypes = {
  cellName: PropTypes.string,
  isEditable: PropTypes.bool.isRequired,
  cell: PropTypes.object.isRequired,
};

export default LayoutCellHeader;
