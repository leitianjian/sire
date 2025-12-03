import React, { Fragment } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import Row from "antd/lib/row";
import Col from "antd/lib/col";
import Button from "antd/lib/button";
import { connect } from "react-redux";
import Select from "antd/lib/select";
import CellBase from "./CellBase";
import {
  getMemoizeCsStarted,
  getWebsocketConnected,
} from "../../state/selectors";
import { sendCmd } from "../../state/actions";
import InputNumber from "antd/lib/input-number";

import SixAxes from "../../images/six_axes.png";
import SevenAxes from "../../images/seven_axes.png";

import { IntlProvider, FormattedMessage } from "react-intl";

// 进行中英文切换
import zhCN from "./locale/zhCN";
import enUS from "./locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

class SetDH extends CellBase {
  name = "构型参数配置";
  width = 24;
  height = 12;

  state = {
    type: "six_axes",
    inputs: {},
  };

  updateInput = (name, value) => {
    const inputs = { ...this.state.inputs };
    inputs[name] = value;
    this.setState({ inputs });
  };

  renderSixAxesInput = (prefixCls) => {
    const { inputs } = this.state;
    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    console.log(SixAxes);
    return (
      <Fragment>
        <Row style={{ height: "500px" }}>
          <Col span={12} className={`${prefixCls}-left`}>
            {["a1", "a2", "a3", "d1", "d3", "d4", "tool0"].map(
              (name, index) => (
                <Row key={name} className={`${prefixCls}-input`}>
                  <span> {name} </span>
                  <InputNumber
                    disabled={ds}
                    step={1}
                    value={inputs[name]}
                    size={"small"}
                    onChange={(value) => {
                      this.updateInput(name, value);
                    }}
                  />
                  <span
                    style={{
                      whiteSpace: "nowrap",
                      display: "inline",
                      paddingLeft: "3px",
                    }}
                  >
                    mm
                  </span>
                </Row>
              )
            )}
          </Col>
          <Col
            span={12}
            className={`${prefixCls}-bg`}
            style={{
              backgroundImage: `url(${SixAxes})`,
              backgroundRepeat: "no-repeat",
              backgroundSize: "contain",
              // width:"60%",
              // height:"100%"
            }}
          >
            {/* <Image src={`${true?SixAxes:SevenAxes}`}/> */}
          </Col>
        </Row>
      </Fragment>
    );
  };

  renderSevenAxesInput = (prefixCls) => {
    const { inputs } = this.state;
    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    return (
      <Fragment>
        <Row style={{ height: "500px" }}>
          <Col span={12} className={`${prefixCls}-left`}>
            {["d1", "d3", "d5", "tool0"].map((name, index) => (
              <Row key={name} className={`${prefixCls}-input`}>
                <span> {name} </span>
                <InputNumber
                  disabled={ds}
                  step={1}
                  value={inputs[name]}
                  size={"small"}
                  onChange={(value) => {
                    this.updateInput(name, value);
                  }}
                />
                <span
                  style={{
                    whiteSpace: "nowrap",
                    display: "inline",
                    paddingLeft: "3px",
                  }}
                >
                  mm
                </span>
              </Row>
            ))}
          </Col>
          <Col
            span={12}
            className={`${prefixCls}-bg`}
            style={{
              backgroundImage: `url(${SevenAxes})`,
              backgroundRepeat: "no-repeat",
              backgroundSize: "contain",
              // width:"60%",
              // height:"100%"
            }}
          >
            {/* <Image src={`${true?SixAxes:SevenAxes}`}/> */}
          </Col>
        </Row>
      </Fragment>
    );
  };

  renderHeader = (prefixCls) => {
    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    return (
      <Fragment>
        <Row>
          <div className={`${prefixCls}-header`}>
            <Button
              disabled={!this.props.isWsConnected || !this.props.isCsStarted}
              type={"primary"}
              className={`${prefixCls}-header-buttons`}
              onClick={() => this.props.sendCmd("cs_stop")}
            >
              开始配置
            </Button>
            <Button
              disabled={!this.props.isWsConnected || !!this.props.isCsStarted}
              type={"primary"}
              className={`${prefixCls}-header-buttons`}
              onClick={() => this.props.sendCmd("cs_start")}
            >
              结束配置
            </Button>
            <Button
              disabled={ds}
              type={"primary"}
              className={`${prefixCls}-header-buttons`}
              onClick={(e) => {
                const { inputs } = this.state;
                this.props.sendCmd("cs_stop");
                let cmd = `setdh --${this.state.type}`;
                Object.keys(inputs).forEach((name) => {
                  cmd += ` --${name}_${this.state.type}=${inputs[name] / 1000}`;
                });
                this.props.sendCmd(cmd);
                this.props.sendCmd("savexml");
              }}
            >
              保存配置
            </Button>
            <Select
              disabled={ds}
              size={"small"}
              placeholder={"选择构型"}
              value={this.state.type}
              onChange={(type) => {
                this.setState({
                  type,
                  inputs: {},
                });
              }}
            >
              <Select.Option value={"six_axes"} key={"six_axes"}>
                <FormattedMessage id="六轴机器人" />
              </Select.Option>
              <Select.Option value={"seven_axes"} key={"seven_axes"}>
                <FormattedMessage id="七轴机器人" />
              </Select.Option>
            </Select>
          </div>
        </Row>
      </Fragment>
    );
  };

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;
    const prefixCls = getPrefixCls("setdh", customizePrefixCls);
    const { type } = this.state;
    return (
      <div className={`${prefixCls}`}>
        {this.renderHeader(prefixCls)}
        {type === "six_axes" && this.renderSixAxesInput(prefixCls)}
        {type === "seven_axes" && this.renderSevenAxesInput(prefixCls)}
        {/* <Row> */}
        {/*    <Col span={12} className={`${prefixCls}-left`}> */}
        {/*        { type==='six_axes' && this.renderSixAxesInput(prefixCls) } */}
        {/*        { type==='seven_axes' && this.renderSevenAxesInput(prefixCls) } */}
        {/*    </Col> */}
        {/*    <Col span={12} className={`${prefixCls}-bg`} */}
        {/*         style={{ */}
        {/*             backgroundImage: `url(${type==='six_axes'?SixAxes:SevenAxes})`, */}
        {/*             width:"60%" */}
        {/*         }} */}
        {/*    > */}
        {/*    </Col> */}
        {/* </Row> */}
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

function mapStateToProps(state, props) {
  return {
    isCsStarted: getMemoizeCsStarted(state),
    isWsConnected: getWebsocketConnected(state),
  };
}

const ConnectedSetDH = connect(mapStateToProps, {
  sendCmd,
})(SetDH);

ConnectedSetDH.NAME = "构型参数配置";
export default ConnectedSetDH;
