import React, { Component } from "react";
import Dashboard from "../components/Dashboard";
import { connect } from "react-redux";
import {
  getCurrentDashboardAndCells,
  getLayoutCols,
  getLayoutContainerPadding,
  getLayoutMargin,
  getLayoutRowHeight,
} from "../state/selectors";
import { updateDashboard, deleteCell } from "../state/actions";

class DashboardPage extends Component {
  onPositionChange = (cells) => {
    // 此处一定要做复制操作，不能直接Return原dashboard和cells
    let { dashboard, updateDashboard } = this.props;
    dashboard = { ...dashboard, cells: [...cells] };
    updateDashboard(dashboard);
  };

  onCellUpdate = (cell) => {
    // 此处一定要做复制操作，不能直接Return原dashboard和cells
    let { dashboard, updateDashboard } = this.props;
    const cells = [...dashboard.cells];
    cells.forEach((c, i) => {
      if (cell.i === c.i) {
        cells[i] = { ...cell };
      }
    });
    dashboard = { ...dashboard, cells };
    updateDashboard(dashboard);
  };

  onDeleteCell = (cellId) => {
    this.props.deleteCell(this.props.dashboard.i, cellId);
  };

  render() {
    const {
      dashboard,
      layoutCols,
      layoutRowHeight,
      layoutMargin,
      layoutContainerPadding,
    } = this.props;
    console.log("test", dashboard);
    return (
      <Dashboard
        dashboard={dashboard}
        layoutCols={layoutCols}
        layoutRowHeight={layoutRowHeight}
        layoutMargin={layoutMargin}
        layoutContainerPadding={layoutContainerPadding}
        inPresentationMode={false}
        onPositionChange={this.onPositionChange}
        onCellUpdate={this.onCellUpdate}
        onDeleteCell={this.onDeleteCell}
        onCloneCell={this.onAddCell}
        onAddCell={this.onAddCell}
        onSummonOverlayTechnologies={() => {}}
        manualRefresh={0}
      />
    );
  }
}

function mapStateToProps(state, props) {
  return {
    dashboard: getCurrentDashboardAndCells(state),
    layoutCols: getLayoutCols(state),
    layoutRowHeight: getLayoutRowHeight(state),
    layoutMargin: getLayoutMargin(state),
    layoutContainerPadding: getLayoutContainerPadding(state),
  };
}

export default connect(mapStateToProps, {
  updateDashboard,
  deleteCell,
})(DashboardPage);
