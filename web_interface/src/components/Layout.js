import React, { Component, Fragment } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import LayoutCell from "./LayoutCell";
import CellByTypes from "../shared/cells";
import { connect } from "react-redux";
import {
  getCommandSendDelay,
  getCommandSendInterval,
} from "../state/selectors";

const Unknown = CellByTypes.Unknown;
class Layout extends Component {
  constructor(props) {
    super(props);
    this.cellByTypes = {};
    Object.keys(CellByTypes).forEach((key) => {
      this.cellByTypes[key.toLowerCase()] = CellByTypes[key];
    });
  }

  renderLayout = ({ getPrefixCls }) => {
    const {
      prefixCls: customizePrefixCls,
      isEditable,
      cell,
      onCloneCell,
      onDeleteCell,
      onCellUpdate,
    } = this.props;
    const prefixCls = getPrefixCls("layout", customizePrefixCls);
    return (
      <LayoutCell
        cell={cell}
        onCellUpdate={onCellUpdate}
        onDeleteCell={onDeleteCell}
        onCloneCell={onCloneCell}
        isEditable={isEditable}
      >
        {this.renderLayoutBody(prefixCls)}
      </LayoutCell>
    );
  };

  renderLayoutBody(prefixCls) {
    const { cell, onCellUpdate, commandSendInterval, commandSendDelay } =
      this.props;
    if (!cell.type || !this.cellByTypes[cell.type.toLowerCase()]) {
      return <Unknown cell={cell} onCellUpdate={onCellUpdate} />;
    } else {
      const CellComponent = this.cellByTypes[cell.type.toLowerCase()];
      if (!CellComponent.CONFIG) {
        return (
          <CellComponent
            cell={cell}
            onCellUpdate={onCellUpdate}
            commandSendInterval={commandSendInterval}
            commandSendDelay={commandSendDelay}
          />
        );
      } else {
        return (
          <Fragment>
            <CellComponent
              cell={cell}
              onCellUpdate={onCellUpdate}
              commandSendInterval={commandSendInterval}
              commandSendDelay={commandSendDelay}
            />
            <CellComponent.CONFIG cell={cell} onCellUpdate={onCellUpdate} />
          </Fragment>
        );
      }
    }
  }

  render() {
    return <ConfigConsumer>{this.renderLayout}</ConfigConsumer>;
  }
}

function mapStateToProps(state, props) {
  return {
    commandSendInterval: getCommandSendInterval(state),
    commandSendDelay: getCommandSendDelay(state),
  };
}

export default connect(mapStateToProps)(Layout);
