import React, { Component } from "react";
import ReactGridLayout, { WidthProvider } from "react-grid-layout";
import { ConfigConsumer } from "antd/lib/config-provider";
import Layout from "./Layout";
import windowSize from "react-window-size";

const GridLayout = WidthProvider(ReactGridLayout);

class LayoutRenderer extends Component {
  handleLayoutChange = (layout) => {
    if (!this.props.onPositionChange) return;

    const self = this;
    let changed = false;
    const newCells = self.props.cells.map((cell) => {
      const l = layout.find((ly) => {
        return ly.i === cell.i;
      });
      if (!l) {
        return cell;
      }
      if (
        cell.x !== l.x ||
        cell.y !== l.y ||
        cell.h !== l.h ||
        cell.w !== l.w
      ) {
        changed = true;
      }
      const newLayout = {
        x: l.x,
        y: l.y,
        h: l.h,
        w: l.w,
      };
      return {
        ...cell,
        ...newLayout,
      };
    });

    if (changed) {
      this.props.onPositionChange(newCells);
    }
  };

  renderLayoutRender = ({ getPrefixCls }) => {
    const {
      prefixCls: customizePrefixCls,
      cells,
      isEditable,
      onDeleteCell,
      onCloneCell,
      onCellUpdate,
      layoutCols,
      layoutMargin,
      layoutContainerPadding,
      windowWidth,
    } = this.props;
    const prefixCls = getPrefixCls("layout-renderer", customizePrefixCls);

    const draggableCls = `.${getPrefixCls(
      "layout-cell",
      customizePrefixCls
    )}--draggable`;
    const rowHeight = Math.floor(windowWidth / layoutCols);
    return (
      <div className={`${prefixCls} ${isEditable ? "editable" : ""}`}>
        <GridLayout
          layout={cells}
          cols={layoutCols}
          rowHeight={rowHeight}
          margin={[layoutMargin, layoutMargin]}
          containerPadding={[layoutContainerPadding, layoutContainerPadding]}
          useCSSTransforms={false}
          onLayoutChange={this.handleLayoutChange}
          draggableHandle={draggableCls}
          isDraggable={true}
          isResizable={true}
        >
          {cells.map((cell) => (
            <div key={cell.i} className={isEditable ? "editable" : ""}>
              <Layout
                cell={cell}
                onDeleteCell={onDeleteCell}
                onCloneCell={onCloneCell}
                onCellUpdate={onCellUpdate}
                isEditable={isEditable}
              />
            </div>
          ))}
        </GridLayout>
      </div>
    );
  };

  render() {
    return <ConfigConsumer>{this.renderLayoutRender}</ConfigConsumer>;
  }
}

export default windowSize(LayoutRenderer);
