import React, { PureComponent } from "react";
import { connect } from "react-redux";
import { IntlProvider, FormattedMessage } from "react-intl";
import Select from "antd/lib/select";
import Modal from "antd/lib/modal";
import Button from "antd/lib/button";
import Menu from "antd/lib/menu";
import Slider from "antd/lib/slider";
import InputNumber from "antd/lib/input-number";
import { ConfigConsumer } from "antd/lib/config-provider";
import {
  BlockOutlined,
  LockOutlined,
  SyncOutlined,
  FullscreenOutlined,
  FullscreenExitOutlined,
} from "@ant-design/icons";

import CreateCellModal from "../components/CreateCellModal";
import {
  actFullScreen,
  actFullScreenInverse,
  actNavigate,
  actUpdateMotionVelPercent,
  actUpdateToolId,
  actUpdateWobjId,
  actToggleCreateCellModal,
  updateDashboard,
  sendCmd,
} from "../state/actions";

const Option = Select.Option;

class Header extends PureComponent {
  constructor(props) {
    super(props);
    this.state = { showChangePageModel: false };
  }

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, prpPathname } = this.props;
    const prefixCls = getPrefixCls("header", customizePrefixCls);

    if (prpPathname === "/" && this.props.menus.length > 0) {
      this.props.dspNavigate(this.props.menus[0].key);
      console.log("navigate");
    }

    let dashboard =
      prpPathname === "/"
        ? null
        : this.props.prpDashboards.find(
            (dash) => dash.i === prpPathname.substr(12)
          );

    console.log(prpPathname);
    console.log(this.props.prpDashboards);

    return (
      <div className={prefixCls}>
        <Menu
          mode="horizontal"
          selectedKeys={[prpPathname]}
          onClick={(e) => {
            this.props.dspNavigate(e.key);
          }}
        >
          {this.props.menus.map((d) => (
            <Menu.Item key={d.key}>
              <FormattedMessage id={d.name} />
            </Menu.Item>
          ))}
        </Menu>
        <div
          className={`${prefixCls}-actions`}
          style={{
            position: "absolute",
            right: 0,
            top: 0,
            display: "inline-block",
          }}
        >
          <Slider
            tooltipVisible={false}
            min={1}
            max={100}
            tipFormatter={(value) => `${value}%`}
            value={this.props.prpMotionVelPercent}
            onChange={(value) => {
              this.props.dspUpdateMotionVelPercent(value);
              this.props.dspSendCmd(`setvel --vel_percent=${value}`);
            }}
          />
          <InputNumber
            min={1}
            max={100}
            size={"small"}
            formatter={(value) => `${value}%`}
            parser={(value) => value.replace("%", "")}
            value={this.props.prpMotionVelPercent}
            onChange={(value) => {
              this.props.dspUpdateMotionVelPercent(value);
              this.props.dspSendCmd(`setvel --vel_percent=${value}`);
            }}
          />
          <Select
            size={"small"}
            placeholder={<FormattedMessage id="工具坐标系" />}
            onChange={this.props.dspUpdateToolId}
            value={this.props.prpToolId}
          >
            {this.props.prpToolIds.map((id) => {
              return (
                <Option value={id} key={id}>
                  <FormattedMessage id="工具坐标系" />
                  {id}
                </Option>
              );
            })}
          </Select>
          <Select
            size={"small"}
            placeholder={<FormattedMessage id="工件坐标系" />}
            onChange={this.props.dspUpdateWobjId}
            value={this.props.prpWobjId}
          >
            {this.props.prpWobjIds.map((id) => {
              return (
                <Option value={id} key={id}>
                  <FormattedMessage id="工件坐标系" />
                  {id}
                </Option>
              );
            })}
          </Select>
          <Select
            showSearch
            style={{ width: 80, float: "right", position: "relative" }}
            onChange={this.changeLocale}
          >
            <Option value="chinese">中文</Option>
            <Option value="english">EN</Option>
          </Select>
          <BlockOutlined
            className={"icon"}
            onClick={() => {
              this.props.dspToggleCreateCellModal();
            }}
          />
          <LockOutlined
            className={`${
              dashboard && dashboard.editable ? "icon" : "icon info"
            }`}
            onClick={() => {
              if (dashboard) {
                dashboard = { ...dashboard, editable: !dashboard.editable };
                this.props.dspUpdateDashboard(dashboard);
              }
            }}
          />
          {!this.props.prpFullscreen && (
            <FullscreenOutlined
              className={"icon"}
              onClick={() => {
                this.props.dspFullScreen();
              }}
            />
          )}
          {this.props.prpFullscreen && (
            <FullscreenExitOutlined
              className={"icon"}
              onClick={() => {
                this.props.dspFullScreenInverse();
              }}
            />
          )}
          <SyncOutlined
            className={"icon"}
            onClick={() => {
              window.location.reload();
            }}
          />
        </div>
        <CreateCellModal prefixCls={prefixCls} />
        <Modal
          visible={this.state.showChangePageModel}
          title={<FormattedMessage id="强制切换页面" />}
          centered={true}
          footer={[
            <Button
              key="close"
              onClick={(e) => {
                this.setState({ showChangePageModel: false });
              }}
            >
              <FormattedMessage id="关闭" />
            </Button>,
            <Button
              key="sure"
              onClick={(e) => {
                this.setState({ showChangePageModel: false });
                this.props.dspSendCmd("cs_stop");
                this.props.dspNavigate(this.props.menus[0].key);
              }}
            >
              <FormattedMessage id="确定" />
            </Button>,
          ]}
        >
          <span className={"danger"}>
            注意：当前控制器是使能/激活状态，如果强制切换到硬件配置界面会使控制器去使能/失活。
          </span>
        </Modal>
      </div>
    );
  };

  render() {
    return (
      <IntlProvider>
        <ConfigConsumer>{this.renderContent}</ConfigConsumer>
      </IntlProvider>
    );
  }
}

function mapStateToProps(state) {
  return {
    prpPathname: state.router.pathname,
    prpFullscreen: state.config.fullscreen,
    prpMotionVelPercent: state.robot.currentMotionVelPercent,
    prpToolIds: state.robot.toolIds,
    prpWobjIds: state.robot.wobjIds,
    prpToolId: state.robot.currentToolId,
    prpWobjId: state.robot.currentWobjId,
    prpIsCsStarted: state.robot.isCsStarted,
    prpDashboards: state.config.dashboards,
  };
}

const mapDispatchToProps = (dispatch) => {
  return {
    dspFullScreen: (...args) => dispatch(actFullScreen(...args)),
    dspFullScreenInverse: (...args) => dispatch(actFullScreenInverse(...args)),
    dspNavigate: (...args) => dispatch(actNavigate(...args)),
    dspUpdateMotionVelPercent: (...args) =>
      dispatch(actUpdateMotionVelPercent(...args)),
    dspUpdateToolId: (...args) => dispatch(actUpdateToolId(...args)),
    dspUpdateWobjId: (...args) => dispatch(actUpdateWobjId(...args)),
    dspToggleCreateCellModal: (...args) =>
      dispatch(actToggleCreateCellModal(...args)),
    dspSendCmd: (...args) => dispatch(sendCmd(...args)),
    dspUpdateDashboard: (...args) => dispatch(updateDashboard(...args)),
  };
};

export default connect(mapStateToProps, mapDispatchToProps)(Header);
