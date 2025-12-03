import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import PropTypes from "prop-types";
import { connect } from "react-redux";
import { WifiOutlined, CheckCircleOutlined } from "@ant-design/icons";
import {
  getStateCode,
  getWebsocketConnected,
  getCsErrMsg,
  getCsErrCode,
  getlocalestate,
} from "../../state/selectors";

// 英文
// import {connect} from 'react-redux'
import zhCN from "./locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "./locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

class RobotStatus extends Component {
  renderJogJoint = ({ getPrefixCls }) => {
    let {
      prefixCls: customizePrefixCls,
      websocketConnected,
      stateCode,
      csErrCode,
      csErrMsg,
    } = this.props;
    if (!stateCode) {
      stateCode = 100;
    }
    const colors = {
      100: "#AAAAAA",
      200: "#FF851B",
      300: "#52c41a",
      400: "#0074D9",
      410: "#ff0000",
      420: "#ff0000",
      500: "#ff0000",
    };
    const names = {
      100: "去使能",
      200: "手动",
      300: "自动",
      400: "运行中",
      410: "暂停中",
      420: "停止",
      500: "错误",
    };
    // console.log("robot state code", stateCode)
    const color = colors[stateCode];
    const name = names[stateCode];
    const prefixCls = getPrefixCls("robot-status", customizePrefixCls);
    return (
      <div className={`${prefixCls}`}>
        {csErrCode && csErrCode !== 0 ? (
          <div
            style={{
              display: "block",
              width: "100%",
              textAlign: "left",
              color: "red",
              wordBreak: "break-all",
              wordWrap: "break-word",
            }}
          >
            系统错误:{csErrMsg}
          </div>
        ) : (
          <div
            style={{
              display: "block",
              width: "100%",
              textAlign: "left",
              color: "#52c41a",
              wordBreak: "break-all",
              wordWrap: "break-word",
            }}
          >
            return_code:{csErrCode}
          </div>
        )}
        <div>
          {!!websocketConnected && <WifiOutlined className={"success"} />}
          {!!websocketConnected && (
            <span style={{ whiteSpace: "nowrap" }}>
              <FormattedMessage id="从站已连接" />
            </span>
          )}
          {!websocketConnected && <WifiOutlined className={"danger"} />}
          {!websocketConnected && (
            <span>
              <FormattedMessage id="未连接" />
            </span>
          )}
        </div>
        <div>
          <CheckCircleOutlined
            style={{
              color,
              fill: color,
            }}
            fill={color}
            type="check-circle"
            className="info"
            theme="filled"
          />
          <span>
            <FormattedMessage id={name || "无状态"} />
          </span>
        </div>
      </div>
    );
  };

  render() {
    const { localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <ConfigConsumer>{this.renderJogJoint}</ConfigConsumer>
      </IntlProvider>
    );
  }
}

RobotStatus.propTypes = {
  cell: PropTypes.object.isRequired,
  stateCode: PropTypes.number,
};

function mapStateToProps(state) {
  return {
    stateCode: getStateCode(state),
    websocketConnected: getWebsocketConnected(state),
    csErrCode: getCsErrCode(state),
    csErrMsg: getCsErrMsg(state),
    localestate: getlocalestate(state),
  };
}

const ConnectedRobotStatus = connect(mapStateToProps)(RobotStatus);

ConnectedRobotStatus.NAME = "机器人状态";
export default ConnectedRobotStatus;
