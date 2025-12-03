import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import DashboardEmpty from "./DashboardEmpty";
import LayoutRenderer from "./LayoutRenderer";

class Dashboard extends Component {
  renderDashboard = ({ getPrefixCls }) => {
    const {
      dashboard,
      prefixCls: customizePrefixCls,
      onCloneCell,
      onDeleteCell,
      onPositionChange,
      onCellUpdate,
      layoutCols,
      layoutRowHeight,
      layoutMargin,
      layoutContainerPadding,
    } = this.props;
    const prefixCls = getPrefixCls("dashboard", customizePrefixCls);
    return (
      <div className={`${prefixCls}`} id={"dashboard"}>
        {!!dashboard && !!dashboard.cells && dashboard.cells.length ? (
          <LayoutRenderer
            layoutCols={layoutCols}
            layoutRowHeight={layoutRowHeight}
            layoutMargin={layoutMargin}
            layoutContainerPadding={layoutContainerPadding}
            cells={dashboard.cells}
            templates={[]}
            manualRefresh={10}
            onCellUpdate={onCellUpdate}
            isEditable={!!dashboard.editable}
            onCloneCell={onCloneCell}
            onDeleteCell={onDeleteCell}
            onPositionChange={onPositionChange}
          />
        ) : (
          <DashboardEmpty dashboard={dashboard} onAddCell={() => {}} />
        )}
      </div>
    );
  };

  render() {
    return <ConfigConsumer>{this.renderDashboard}</ConfigConsumer>;
  }
}

export default Dashboard;
