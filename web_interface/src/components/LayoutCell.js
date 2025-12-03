import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import LayoutCellMenu from "./LayoutCellMenu";
import LayoutCellHeader from "./LayoutCellHeader";

class LayoutCell extends Component {
  renderLayoutCell = ({ getPrefixCls }) => {
    const {
      prefixCls: customizePrefixCls,
      isEditable,
      cell,
      onCloneCell,
      onCellUpdate,
      onDeleteCell,
      children,
    } = this.props;

    const prefixCls = getPrefixCls("layout-cell", customizePrefixCls);
    return (
      <div
        className={`${prefixCls}`}
        ref={(ref) => {
          this.container = ref;
        }}
      >
        <LayoutCellMenu
          cell={cell}
          isEditable={isEditable}
          dataExists={false}
          onClone={onCloneCell}
          onDelete={onDeleteCell}
          onCellUpdate={onCellUpdate}
          getPopupContainer={() => {
            return this.container;
          }}
        />
        <LayoutCellHeader
          cell={cell}
          cellName={cell.name}
          isEditable={isEditable}
        />
        <div className={`${prefixCls}-body`}>{children}</div>
      </div>
    );
  };

  render() {
    return <ConfigConsumer>{this.renderLayoutCell}</ConfigConsumer>;
  }
}

export default LayoutCell;
