import React, { Fragment, Component } from "react";
import { connect } from "react-redux";
import Row from "antd/lib/row";
import Col from "antd/lib/col";

import {
  getCurrentCommand,
  getStateCode,
  getCurrentPlan,
  getlocalestate,
  getWebsocketConnected,
  getMemoizeCsErrCode,
  getMemoizeCsErrMsg,
} from "../state/selectors";

import { ConfigConsumer } from "antd/lib/config-provider";
import {
  WifiOutlined,
  SettingFilled,
  CodeOutlined,
  ClockCircleOutlined,
  BulbOutlined,
  MessageOutlined,
  EditOutlined,
  PlayCircleOutlined,
  CheckCircleOutlined,
  PauseCircleOutlined,
  StopOutlined,
  ExclamationCircleOutlined,
} from "@ant-design/icons";
import Tooltip from "antd/lib/tooltip";

import zhCN from "./locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "./locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

class Footer extends Component {
  renderLeftCorner = (prefixCls) => {
    const { currentCommand } = this.props;

    return (
      <Fragment>
        <Col span={18}>
          <div
            className={`${prefixCls}-command ${
              currentCommand.return_code ? "error" : "success"
            }`}
          >
            <Row>
              <Col span={9}>
                <div
                  className={`${prefixCls}-text ${
                    currentCommand.isRunning ? prefixCls + "-cmd-running" : ""
                  }`}
                >
                  <Tooltip
                    placement="top"
                    title={`发送指令：${currentCommand.cmd}`}
                  >
                    <CodeOutlined className={`${prefixCls}-icon`} />
                  </Tooltip>
                  {currentCommand.cmd ? currentCommand.cmd : ""}
                </div>
              </Col>
              <Col span={2}>
                <div className={`${prefixCls}-text`}>
                  <Tooltip
                    placement="top"
                    title={`返回时间：${currentCommand.cmdResponseTime} ms`}
                  >
                    <ClockCircleOutlined className={`${prefixCls}-icon`} />
                  </Tooltip>
                  {currentCommand.cmdResponseTime
                    ? currentCommand.cmdResponseTime + " ms"
                    : ""}
                </div>
              </Col>
              <Col span={2}>
                <div className={`${prefixCls}-text`}>
                  <Tooltip
                    placement="top"
                    title={`返回代码：${currentCommand.return_code}`}
                  >
                    <BulbOutlined className={`${prefixCls}-icon`} />
                  </Tooltip>
                  {currentCommand.return_code ? currentCommand.return_code : ""}
                </div>
              </Col>
              <Col span={9}>
                <div className={`${prefixCls}-text`}>
                  <Tooltip
                    placement="top"
                    title={`返回消息：${currentCommand.return_message}`}
                  >
                    <MessageOutlined className={`${prefixCls}-icon`} />
                  </Tooltip>
                  {currentCommand.return_message
                    ? currentCommand.return_message
                    : ""}
                </div>
              </Col>
            </Row>
          </div>
        </Col>
      </Fragment>
    );
  };

  renderRightCorner = (prefixCls) => {
    let { websocketConnected, stateCode, currentPlan, csErrMsg, csErrCode } =
      this.props;

    const states = {
      0: {
        color: "#AAAAAA",
        info: "无状态",
        icon: (props) => <ClockCircleOutlined {...props} />,
        tooltip: "无状态",
      },
      100: {
        color: "#AAAAAA",
        info: "去使能",
        icon: (props) => <ClockCircleOutlined {...props} />,
        tooltip: "去使能",
      },
      200: {
        color: "#FF851B",
        info: "手动",
        icon: (props) => <EditOutlined {...props} />,
        tooltip: "手动",
      },
      300: {
        color: "#52c41a",
        info: "自动",
        icon: (props) => <CheckCircleOutlined {...props} />,
        tooltip: "自动",
      },
      400: {
        color: "#0074D9",
        info: "运行中",
        icon: (props) => <PlayCircleOutlined {...props} />,
        tooltip: "运行中",
      },
      410: {
        color: "#ff0000",
        info: "暂停中",
        icon: (props) => <PauseCircleOutlined {...props} />,
        tooltip: "暂停中",
      },
      420: {
        color: "#ff0000",
        info: "停止",
        icon: (props) => <StopOutlined {...props} />,
        tooltip: "停止",
      },
      500: {
        color: "#ff0000",
        info: "错误",
        icon: (props) => <ExclamationCircleOutlined {...props} />,
        tooltip: `${csErrCode}:${csErrMsg}`,
      },
    };

    const state = states[csErrCode !== 0 ? 500 : stateCode]
      ? states[csErrCode !== 0 ? 500 : stateCode]
      : states[0];

    currentPlan = !!currentPlan && currentPlan !== "none";

    return (
      <Fragment>
        <Col span={6} className={`${prefixCls}-status`}>
          <div>
            <Tooltip placement="top" title={"机器人连接状态"}>
              <WifiOutlined
                className={websocketConnected ? "icon success" : "icon danger"}
              />
            </Tooltip>
            <span>
              <FormattedMessage id={websocketConnected ? "已连接" : "未连接"} />
            </span>
          </div>
          <div className={"info"}>
            <Tooltip placement="top" title={state.tooltip}>
              {state.icon({
                style: { color: state.color, fill: state.color },
                fill: state.color,
                className: "icon info",
              })}
            </Tooltip>
            <FormattedMessage id={state.info} />
          </div>
          <div className={currentPlan ? "success" : "disabled"}>
            <Tooltip placement="top" title={"Current Plan"}>
              <SettingFilled
                style={{
                  color: state.color,
                  fill: state.color,
                }}
                fill={state.color}
                className={"icon"}
                spin={!!currentPlan}
              />
            </Tooltip>
            {currentPlan ? (
              <FormattedMessage id="运动中" />
            ) : (
              <FormattedMessage id="停止" />
            )}
          </div>
        </Col>
      </Fragment>
    );
  };

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;
    const prefixCls = getPrefixCls("footer", customizePrefixCls);

    return (
      <div className={prefixCls}>
        <Row>
          {this.renderLeftCorner(prefixCls)}
          {this.renderRightCorner(prefixCls)}
        </Row>
      </div>
    );
  };

  render() {
    const { localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <ConfigConsumer>{this.renderContent}</ConfigConsumer>
      </IntlProvider>
    );
  }
}

function mapStateToProps(state) {
  return {
    currentCommand: getCurrentCommand(state),
    websocketConnected: getWebsocketConnected(state),
    stateCode: getStateCode(state),
    currentPlan: getCurrentPlan(state),
    csErrCode: getMemoizeCsErrCode(state),
    csErrMsg: getMemoizeCsErrMsg(state),
    localestate: getlocalestate(state),
  };
}

export default connect(mapStateToProps)(Footer);
