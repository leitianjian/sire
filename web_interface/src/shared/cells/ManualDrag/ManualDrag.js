import React from "react";
import { IntlProvider } from "react-intl";
import { ConfigConsumer } from "antd/lib/config-provider";
import Button from "antd/lib/button";
import { connect } from "react-redux";
import CellBase from "../CellBase";
import { sendCmd } from "../../../state/actions";
import {
  getMemoizeCsStarted,
  getStateCode,
  getWebsocketConnected,
} from "../../../state/selectors";

// 组件内容 //
class ManualDrag extends CellBase {
  constructor(props) {
    super(props);
    this.handleChange = this.handleChange.bind(this);
    this.handleBlur = this.handleBlur.bind(this);
  }

  handleChange(e) {
    console.log("single-step click");
    this.props.sendCmd(
      "djmode --djmode=1 --djstep=" + e.target.value,
      (msg) => {
        console.log(msg.data);
      }
    );
  }

  handleBlur(e) {
    console.log("single-step click");
    this.props.sendCmd(
      "djmode --djmode=1 --djstep=" + func(e.target.value),
      (msg) => {
        console.log(msg.data);
      }
    );
  }

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;
    const prefixCls = getPrefixCls("manual-drag", customizePrefixCls);

    const direction = this.props.dragDirection ? this.props.dragDirection : "";

    const ds =
      !this.props.isWsConnected ||
      !this.props.isCsStarted ||
      this.props.stateCode !== 200;

    return (
      <div>
        <div className={`${prefixCls}-manual-drag`}>
          <span style={{ display: "inline-block", fontSize: "14px" }}>
            拖拽方向：
          </span>
          <Button
            className={
              dragDirection(direction)[0]
                ? `${prefixCls}-btn-on`
                : `${prefixCls}-btn1`
            }
            type="primary"
            disabled={ds}
            onClick={() => {
              console.log("x-direction click");
              this.props.sendCmd(
                "fcd --x=1 --y=0 --z=0 --a=0 --b=0 --c=0",
                (msg) => {
                  console.log(msg.data);
                }
              );
            }}
          >
            X轴向
          </Button>

          <Button
            className={
              dragDirection(direction)[1]
                ? `${prefixCls}-btn-on`
                : `${prefixCls}-btn1`
            }
            type="primary"
            disabled={ds}
            onClick={() => {
              console.log("y-direction click");
              this.props.sendCmd(
                "fcd --x=0 --y=1 --z=0 --a=0 --b=0 --c=0",
                (msg) => {
                  console.log(msg.data);
                }
              );
            }}
          >
            Y轴向
          </Button>

          <Button
            className={
              dragDirection(direction)[2]
                ? `${prefixCls}-btn-on`
                : `${prefixCls}-btn1`
            }
            type="primary"
            disabled={ds}
            onClick={() => {
              console.log("z-direction click");
              this.props.sendCmd(
                "fcd --x=0 --y=0 --z=1 --a=0 --b=0 --c=0",
                (msg) => {
                  console.log(msg.data);
                }
              );
            }}
          >
            z轴向
          </Button>

          <Button
            className={
              dragDirection(direction)[3]
                ? `${prefixCls}-btn-on`
                : `${prefixCls}-btn1`
            }
            type="primary"
            disabled={ds}
            onClick={() => {
              console.log("all-direction click");
              this.props.sendCmd(
                "fcd --x=1 --y=1 --z=1 --a=1 --b=1 --c=1",
                (msg) => {
                  console.log(msg.data);
                }
              );
            }}
          >
            无限制
          </Button>
        </div>

        <div>
          <div style={{ marginTop: "25px" }}>
            <span style={{ display: "inline-block", fontSize: "14px" }}>
              拖拽模式：
            </span>
            <Button
              className={
                !this.props.dragMode
                  ? `${prefixCls}-btn-on`
                  : `${prefixCls}-btn2`
              }
              type="primary"
              disabled={ds}
              onClick={() => {
                console.log("continuous click");
                this.props.sendCmd("djmode --djmode=0", (msg) => {
                  console.log(msg.data);
                });
              }}
            >
              连续模式
            </Button>
          </div>

          <div style={{ marginTop: "10px" }}>
            <Button
              className={
                this.props.dragMode
                  ? `${prefixCls}-btn-on`
                  : `${prefixCls}-btn2`
              }
              style={{ marginLeft: "73px" }}
              type="primary"
              disabled={ds}
              onClick={() => {
                console.log("single-step click");
                this.props.sendCmd(
                  "djmode --djmode=1 --djstep=" + this.props.dragStep,
                  (msg) => {
                    console.log(msg.data);
                  }
                );
              }}
            >
              单步模式
            </Button>
            <form style={{ display: "inline-block" }}>
              <label style={{ fontSize: "12px", marginLeft: "5px" }}>
                步长：
                <input
                  className={`${prefixCls}-inp1`}
                  type="range"
                  value={this.props.dragStep ? this.props.dragStep : "1"}
                  disabled={ds || !this.props.dragMode}
                  min="1"
                  max="100"
                  onChange={this.handleChange}
                />
                <input
                  className={`${prefixCls}-inp2`}
                  type="number"
                  value={this.props.dragStep ? this.props.dragStep : "1"}
                  disabled={ds || !this.props.dragMode}
                  onChange={this.handleBlur}
                />
                <span style={{ verticalAlign: "middle", marginLeft: "5px" }}>
                  /10&nbsp;mm
                </span>
              </label>
            </form>
          </div>
        </div>
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

function func(props) {
  const dist = parseInt(props);
  if (dist > 0 && dist < 101) {
    return dist.toString();
  } else if (dist < 1) {
    return "1";
  } else if (dist > 100) {
    return "100";
  } else {
    return "1";
  }
}

// 拖拽方向判断//
function dragDirection(props) {
  let direct = [false, false, false, false];
  let flag = 0;

  for (let i = 0; i < 6; i++) {
    if (props[i] === 1) {
      flag++;
    }
  }

  if (flag === 1) {
    if (props[0] === 1 && props[1] === 0 && props[2] === 0) {
      direct = [true, false, false, false];
    } else if (props[0] === 0 && props[1] === 1 && props[2] === 0) {
      direct = [false, true, false, false];
    } else if (props[0] === 0 && props[1] === 0 && props[2] === 1) {
      direct = [false, false, true, false];
    }
  } else if (flag === 6) {
    if (props[0] === 1 && props[1] === 1 && props[2] === 1) {
      direct = [false, false, false, true];
    }
  }

  return direct;
}

// 拖拽模式判断//

// 组件用到的属性 //
function mapStateToProps(state) {
  return {
    dragDirection: state.server.dragDirection,
    dragMode: state.server.dragMode,
    dragStep: state.server.dragStep,
    stateCode: getStateCode(state),
    isCsStarted: getMemoizeCsStarted(state),
    isWsConnected: getWebsocketConnected(state),
  };
}

// 组件用到的方法 //
function mapDispatchToProps(dispatch) {
  return {
    sendCmd: (...args) => dispatch(sendCmd(...args)),
  };
}

// 连接组件类与属性和方法 //
const ConnectedManualDrag = connect(
  mapStateToProps,
  mapDispatchToProps
)(ManualDrag);

ConnectedManualDrag.NAME = "力控拖拽设置";

export default ConnectedManualDrag;
