import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import PropTypes from "prop-types";
import Menu from "antd/lib/menu";
import Dropdown from "antd/lib/dropdown";
import { EditFilled } from "@ant-design/icons";
import CellByTypes from "../shared/cells";

import { connect } from "react-redux";
import { createSelector } from "reselect";
import { getlocalestate } from "../state/selectors";
import zhCN from "../shared/cells/locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "../shared/cells/locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

const MenuItem = Menu.Item;

class LayoutCellMenu extends Component {
  constructor(props) {
    super(props);
    this.state = {
      subMenuIsOpen: false,
    };
    const self = this;
    self.cellByTypes = {};
    Object.keys(CellByTypes).forEach((key) => {
      self.cellByTypes[key.toLowerCase()] = CellByTypes[key];
    });
  }

  getOptions = () => {
    const { cell } = this.props;
    return { ...cell.options };
  };

  updateOptions = (options) => {
    const cell = { ...this.props.cell };
    cell.options = { ...cell.options, ...options };
    this.props.onCellUpdate(cell);
  };

  handleConfigCell = () => {
    this.updateOptions({
      ...this.getOptions(),
      showConfig: true,
    });
  };

  handleDeleteCell = () => {
    const { onDelete, cell } = this.props;
    onDelete(cell.i);
  };

  handleCloneCell = () => {
    let { onClone, cell } = this.props;
    cell = { ...cell };
    cell.y = cell.y + 1;
    onClone(cell);
  };

  editMenu = (prefixCls) => {
    const self = this;
    const { cell, onCellUpdate } = this.props;
    const Config =
      cell && cell.type && this.cellByTypes[cell.type.toLowerCase()]
        ? this.cellByTypes[cell.type.toLowerCase()].CONFIG
        : null;
    const menu = (
      <Menu
        onClick={(e) => {
          switch (e.key) {
            case "config":
              self.handleConfigCell();
              break;
            case "clone":
              self.handleCloneCell();
              break;
            case "delete":
              self.handleDeleteCell();
              break;
            default:
              break;
          }
        }}
      >
        {!!Config && (
          <MenuItem key="config">
            <FormattedMessage id="配置" />
            <Config cell={cell} onCellUpdate={onCellUpdate} />
          </MenuItem>
        )}
        <MenuItem key="clone">
          <FormattedMessage id="克隆" />
        </MenuItem>
        <MenuItem key="delete">
          <FormattedMessage id="删除" />
        </MenuItem>
      </Menu>
    );
    return (
      <Dropdown
        overlay={menu}
        trigger={["click"]}
        placement={"bottomRight"}
        className={`${prefixCls}--button`}
        overlayClassName={`${prefixCls}--dropdown-menu`}
        getPopupContainer={self.props.getPopupContainer}
      >
        <EditFilled />
      </Dropdown>
    );
  };

  renderMenu = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, isEditable } = this.props;
    if (!isEditable) {
      return;
    }
    const prefixCls = getPrefixCls("layout-cell-context", customizePrefixCls);
    return <div className={`${prefixCls}`}>{this.editMenu(prefixCls)}</div>;
  };

  render() {
    const { localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <ConfigConsumer>{this.renderMenu}</ConfigConsumer>
      </IntlProvider>
    );
  }
}

LayoutCellMenu.propTypes = {
  cell: PropTypes.object.isRequired,
  isEditable: PropTypes.bool.isRequired,
  getPopupContainer: PropTypes.func.isRequired,
  onClone: PropTypes.func.isRequired,
  onDelete: PropTypes.func.isRequired,
  onCellUpdate: PropTypes.func.isRequired,
};

const getState = createSelector(getlocalestate, (localestate) => {
  return {
    localestate,
  };
});

function mapStateToProps(state, props) {
  return getState(state, props);
}

export default connect(mapStateToProps)(LayoutCellMenu);
