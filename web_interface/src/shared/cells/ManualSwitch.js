import React from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import { connect } from "react-redux";
import Switch from "antd/lib/switch";
import Modal from "antd/lib/modal";
import Button from "antd/lib/button";
import { RollbackOutlined, FlagOutlined } from "@ant-design/icons";
import CellBase from "./CellBase";
import {
  getlocalestate,
  getStateCode,
  getCsErrCode,
  getCsErrMsg,
  getMemoizeCsStarted,
  getWebsocketConnected,
  hashEqualSelectorCreator,
} from "../../state/selectors";
import { dispatchAutoState, sendCmd } from "../../state/actions";

// 英文
import zhCN from "./locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "./locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

class ManualSwitch extends CellBase {
  constructor(props) {
    super(props);
    this.backHomeModalTrue = this.backHomeModalTrue.bind(this);
    this.backHomeModalFalse = this.backHomeModalFalse.bind(this);
  }

  state = {
    enable: false,
    checkState: false,
    isEnableModel: false,
    isAuto: false,
    saveHomeModal: false,
    backHomeModal: false,
  };

  onChange = (checked) => {
    if (checked) {
      this.props.sendCmd("md");
      this.props.sendCmd("en");
      this.props.sendCmd("rc");
    } else {
      this.props.sendCmd("ds");
    }
  };

  saveHomeModalTrue = () => {
    this.props.sendCmd("savehome");
    this.setState({
      saveHomeModal: false,
    });
  };

  saveHomeModalFalse = () => {
    this.setState({
      saveHomeModal: false,
    });
  };

  backHomeModalTrue() {
    if (!this.state.isAuto) {
      this.props.sendCmd("rs");
    }
    this.setState({ backHomeModal: false });
  }

  backHomeModalFalse() {
    this.setState({ backHomeModal: false });
  }

  changeAuto = () => {
    const { stateCode, dispatchAutoState } = this.props;
    if (!!stateCode && stateCode === 300) {
      this.props.sendCmd("program --set_manual");
      dispatchAutoState("0");
      this.setState({
        isAuto: false,
      });
    } else if (!!stateCode && stateCode === 200) {
      this.props.sendCmd("program --set_auto");
      dispatchAutoState("1");
      this.setState({
        isAuto: true,
      });
    }
  };

  renderManualSwitch = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, stateCode } = this.props;

    // const states ={
    //   0:{color: "#AAAAAA", info: "无状态", icon : (props)=><ClockCircleOutlined {...props}/>,       tooltip: "无状态"},
    //   100:{color: "#AAAAAA", info: "去使能", icon : (props)=><ClockCircleOutlined {...props}/>,       tooltip: "去使能"},
    //   200:{color: "#FF851B", info: "手动",   icon : (props)=><EditOutlined {...props}/>,              tooltip: "手动"},
    //   300:{color: "#52c41a", info: "自动",   icon : (props)=><CheckCircleOutlined {...props}/>,       tooltip: "自动"},
    //   400:{color: "#0074D9", info: "运行中", icon : (props)=><PlayCircleOutlined {...props}/>,        tooltip: "运行中"},
    //   410:{color: "#ff0000", info: "暂停中", icon : (props)=><PauseCircleOutlined {...props}/>,       tooltip: "暂停中"},
    //   420:{color: "#ff0000", info: "停止",   icon : (props)=><StopOutlined {...props}/>,              tooltip: "停止"},
    //   500:{color: "#ff0000", info: "错误",   icon : (props)=><ExclamationCircleOutlined {...props}/>, tooltip: `${cs_err_code}:${cs_err_msg}`},
    // };

    const ds = !this.props.isWsConnected || !this.props.isCsStarted;

    const fadeButton = stateCode === 400 || stateCode === 410;
    const prefixCls = getPrefixCls("manual-switch", customizePrefixCls);
    const showEnableModel = false;
    const showDisableModel = false;
    return (
      <div className={`${prefixCls}`}>
        <div>
          <Switch
            checkedChildren={<FormattedMessage id="开" />}
            unCheckedChildren={<FormattedMessage id="关" />}
            checked={!!stateCode && !(stateCode === 100)}
            onChange={this.onChange}
            disabled={ds && !!fadeButton}
          />
          <span>
            {!!stateCode && !(stateCode === 100) ? (
              <FormattedMessage id="使能" />
            ) : (
              <FormattedMessage id="未使能" />
            )}
          </span>
        </div>
        <div>
          <Switch
            checkedChildren={<FormattedMessage id="自动" />}
            unCheckedChildren={<FormattedMessage id="手动" />}
            checked={!!stateCode && (stateCode === 300 || stateCode === 400)}
            disabled={ds && !!fadeButton}
            onChange={this.changeAuto}
          />
          <span>
            {!!stateCode && (stateCode === 300 || stateCode === 400) ? (
              <FormattedMessage id="自动" />
            ) : (
              <FormattedMessage id="手动" />
            )}
          </span>
        </div>
        <div>
          <Button
            disabled={ds}
            size={"large"}
            icon={<RollbackOutlined />}
            onClick={() => {
              this.props.sendCmd("cl");
              this.props.sendCmd("rc");
            }}
          />
          <span>
            <FormattedMessage id="清除错误" />
          </span>
        </div>
        <div>
          <Button
            disabled={ds || stateCode !== 200}
            size={"large"}
            icon={<RollbackOutlined />}
            onClick={() => {
              this.setState({ backHomeModal: true });
            }}
          />
          <span>回零点</span>
        </div>
        <div>
          <Button
            disabled={ds || stateCode !== 100}
            size={"large"}
            icon={<FlagOutlined />}
            onClick={() => {
              this.setState({
                saveHomeModal: true,
              });
            }}
          />
          <span>设置零点</span>
        </div>

        <Modal
          visible={showEnableModel}
          title={<FormattedMessage id="使能错误" />}
          onOk={this.resetEnable}
          onCancel={this.resetEnable}
          centered={true}
          footer={[
            <Button key="close" onClick={this.resetEnable}>
              <FormattedMessage id="关闭" />
            </Button>,
          ]}
        ></Modal>
        <Modal
          visible={showDisableModel}
          title={<FormattedMessage id="去使能错误" />}
          onOk={this.resetDisable}
          onCancel={this.resetDisable}
          centered={true}
          footer={[
            <Button key="close" onClick={this.resetDisable}>
              <FormattedMessage id="关闭" />
            </Button>,
          ]}
        ></Modal>

        <Modal
          title="添加点确认"
          visible={this.state.backHomeModal}
          onOk={this.backHomeModalTrue}
          onCancel={this.backHomeModalFalse}
          centered={true}
          cancelText="取消"
          okText="确定"
        >
          <span className={"danger"}>
            控制器会将机器人退回零点，是否确认继续操作？
          </span>
        </Modal>

        <Modal
          visible={this.state.saveHomeModal}
          title={<FormattedMessage id="设置零点" />}
          onOk={this.saveHomeModalTrue}
          onCancel={this.saveHomeModalFalse}
          centered={true}
          footer={[
            <Button key="close" onClick={this.saveHomeModalFalse}>
              <FormattedMessage id="关闭" />
            </Button>,
            <Button key="close" onClick={this.saveHomeModalTrue}>
              <FormattedMessage id="确认" />
            </Button>,
          ]}
        >
          <span className={"danger"}>
            控制器会将当前位姿设置为零点，是否确认继续操作？
          </span>
        </Modal>
      </div>
    );
  };

  render() {
    const { localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <ConfigConsumer>{this.renderManualSwitch}</ConfigConsumer>
      </IntlProvider>
    );
  }
}

const makeMapStateToProps = () => {
  const getMemoizeResult = hashEqualSelectorCreator(
    getlocalestate,
    getStateCode,
    getCsErrCode,
    getCsErrMsg,
    getMemoizeCsStarted,
    getWebsocketConnected,
    (
      localestate,
      stateCode,
      csErrCode,
      csErrMsg,
      isCsStarted,
      isWsConnected
    ) => ({
      localestate,
      stateCode,
      csErrCode,
      csErrMsg,
      isCsStarted,
      isWsConnected,
    })
  );

  return (state, props) => {
    return getMemoizeResult(state, props);
  };
};

const ConnectedManualSwitch = connect(makeMapStateToProps, {
  dispatchAutoState,
  sendCmd,
})(ManualSwitch);

ConnectedManualSwitch.NAME = "手动控制开关";

export default ConnectedManualSwitch;
