import React from "react";
import { IntlProvider } from "react-intl";
import { ConfigConsumer } from "antd/lib/config-provider";
import Button from "antd/lib/button";
import Switch from "antd/lib/switch";
import Modal from "antd/lib/modal";
import { connect } from "react-redux";
import CellBase from "../CellBase";
import { sendCmd } from "../../../state/actions";
import {
  getMemoizeCsStarted,
  getStateCode,
  getWebsocketConnected,
} from "../../../state/selectors";

// 组件内容 //
class Recording extends CellBase {
  constructor(props) {
    super(props);
    this.state = {
      value: "",
      visible1: false,
      visible2: false,
      mainfun: "",
      subfun: "",
      pos: "",
      theta: "",
    };

    this.handleChange = this.handleChange.bind(this);
    this.handleClick1 = this.handleClick1.bind(this);
    this.handleClick2 = this.handleClick2.bind(this);
    this.addPointOk = this.addPointOk.bind(this);
    this.addPointCancel = this.addPointCancel.bind(this);
    this.removePointOk = this.removePointOk.bind(this);
    this.removePointCancel = this.removePointCancel.bind(this);
  }

  handleChange(e) {
    console.log("mode selection");
    this.props.sendCmd("tmode --mode=" + e.target.value, (msg) => {
      console.log(msg.data);
    });
  }

  handleClick1(e) {
    if (!this.props.isForceControl) {
      console.log("force control start");
      this.props.sendCmd("mvJoint", (msg) => {
        console.log(msg.data);
      });
    } else {
      console.log("force control stop");
      this.props.sendCmd("FCStop", (msg) => {
        console.log(msg.data);
      });
    }
  }

  handleClick2(e) {
    if (!this.props.teachingState) {
      console.log("teaching start");
      this.props.sendCmd("teaching --fun=1", (msg) => {
        console.log(msg.data);
      });
    } else {
      console.log("teaching stop");
      this.props.sendCmd("teaching --fun=4", (msg) => {
        console.log(msg.data);
      });
    }
  }

  showModal1 = () => {
    this.setState({ visible1: true });
  };

  addPointOk() {
    console.log("add a mvj/mvl/mvc point");
    this.props.sendCmd(
      "teaching --fun=2 --movemode=" + this.state.value,
      (msg) => {
        console.log(msg.data);
      }
    );
    this.setState({ visible1: false });
  }

  addPointCancel() {
    console.log("add a mvj/mvl/mvc point cancelled");
    this.setState({ visible1: false });
  }

  showModal2 = () => {
    this.setState({ visible2: true });
  };

  removePointOk() {
    console.log("remove the last point");
    this.props.sendCmd("teaching --fun=3", (msg) => {
      console.log(msg.data);
    });
    this.setState({ visible2: false });
  }

  removePointCancel() {
    console.log("remove the last point cancelled");
    this.setState({ visible2: false });
  }

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;
    const prefixCls = getPrefixCls("recording-mode", customizePrefixCls);

    const mode = this.props.teachingMode;
    let record;

    const ds =
      !this.props.isWsConnected ||
      !this.props.isCsStarted ||
      this.props.stateCode !== 200;

    if (mode === 0) {
      record = (
        <div>
          <div>
            <Button
              className={`${prefixCls}-btn`}
              disabled={ds || !!this.props.isForceControl}
              type="primary"
              onClick={() => {
                console.log("track review in UI");
                this.props.sendCmd("mvf", (msg) => {
                  console.log(msg.data);
                });
              }}
            >
              UI轨迹复现
            </Button>
          </div>
        </div>
      );
    } else if (mode === 1) {
      record = (
        <div>
          <div>
            <form>
              <Button
                className={`${prefixCls}-btn`}
                disabled={ds || !!this.props.isForceControl}
                type="primary"
                style={{ padding: "2px" }}
                onClick={() => {
                  console.log("teaching program generation");
                  this.props.sendCmd(
                    "genfun --mainfun=" +
                      this.state.mainfun +
                      " --subfun=" +
                      this.state.subfun +
                      " --pos=" +
                      this.state.pos +
                      " --theta=" +
                      this.state.theta,
                    (msg) => {
                      console.log(msg.data);
                    }
                  );
                }}
              >
                生成示教程序
              </Button>

              <input
                className={`${prefixCls}-inp`}
                type="text"
                value={this.state.mainfun}
                placeholder="主函数名"
                disabled={ds || !!this.props.isForceControl}
                onChange={(e) => {
                  this.setState({ mainfun: e.target.value });
                }}
              />
              <input
                className={`${prefixCls}-inp`}
                type="text"
                value={this.state.subfun}
                placeholder="子函数名"
                disabled={ds || !!this.props.isForceControl}
                onChange={(e) => {
                  this.setState({ subfun: e.target.value });
                }}
              />
              <input
                className={`${prefixCls}-inp`}
                type="number"
                value={this.state.pos}
                placeholder="间距"
                disabled={ds || !!this.props.isForceControl}
                onChange={(e) => {
                  this.setState({ pos: e.target.value });
                }}
              />
              <input
                className={`${prefixCls}-inp`}
                type="number"
                value={this.state.theta}
                placeholder="角度"
                disabled={ds || !!this.props.isForceControl}
                onChange={(e) => {
                  this.setState({ theta: e.target.value });
                }}
              />
            </form>
          </div>
        </div>
      );
    } else {
      record = (
        <div>
          <div>
            <span style={{ fontSize: "14px", marginLeft: "-56px" }}>
              添加点：
            </span>
            <Button
              className={`${prefixCls}-btnp`}
              disabled={ds || !!this.props.isMvc || !this.props.teachingState}
              type="primary"
              onClick={() => {
                this.setState({ value: "0" });
                this.showModal1();
              }}
            >
              mvj
            </Button>
            <Button
              className={`${prefixCls}-btnp`}
              disabled={ds || !!this.props.isMvc || !this.props.teachingState}
              type="primary"
              onClick={() => {
                this.setState({ value: "1" });
                this.showModal1();
              }}
            >
              mvl
            </Button>
            <Button
              className={`${prefixCls}-btnp`}
              disabled={ds || !this.props.teachingState}
              type="primary"
              onClick={() => {
                this.setState({ value: "2" });
                this.showModal1();
              }}
            >
              mvc
            </Button>

            <Button
              className={`${prefixCls}-btn`}
              disabled={ds || !this.props.teachingState}
              style={{ marginLeft: "10px" }}
              type="primary"
              onClick={this.showModal2}
            >
              删除上个点
            </Button>
          </div>

          <div style={{ marginTop: "20px" }}>
            <form>
              <Button
                className={`${prefixCls}-btn`}
                disabled={ds || !!this.props.isForceControl}
                type="primary"
                style={{ padding: "2px" }}
                onClick={() => {
                  console.log("teaching program generation");
                  this.props.sendCmd(
                    "genfun --mainfun=" +
                      this.state.mainfun +
                      " --subfun=" +
                      this.state.subfun,
                    (msg) => {
                      console.log(msg.data);
                    }
                  );
                }}
              >
                生成示教程序
              </Button>

              <input
                className={`${prefixCls}-inp`}
                type="text"
                value={this.state.mainfun}
                placeholder="主函数名"
                disabled={ds || !!this.props.isForceControl}
                onChange={(e) => {
                  this.setState({ mainfun: e.target.value });
                }}
              />
              <input
                className={`${prefixCls}-inp`}
                type="text"
                value={this.state.subfun}
                placeholder="子函数名"
                disabled={ds || !!this.props.isForceControl}
                onChange={(e) => {
                  this.setState({ subfun: e.target.value });
                }}
              />
            </form>
          </div>
        </div>
      );
    }

    return (
      <div
        style={{
          width: "100%",
          height: "100%",
          paddingLeft: "30px",
          paddingTop: "30px",
        }}
      >
        <form>
          <label style={{ fontSize: "14px", verticalAlign: "center" }}>
            模式选择：
            <select
              className={`${prefixCls}-select`}
              value={this.props.teachingMode ? this.props.teachingMode : "0"}
              disabled={ds || !!this.props.isForceControl}
              onChange={this.handleChange}
            >
              <option value="0">轨迹复现模式</option>
              <option value="1">自动记录模式</option>
              <option value="2">手动记录模式</option>
            </select>
          </label>

          <span style={{ marginLeft: "42px", fontSize: "14px" }}>力控：</span>
          <Switch
            checkedChildren="启动"
            unCheckedChildren="终止"
            checked={this.props.isForceControl}
            className={`${prefixCls}-switch`}
            onClick={this.handleClick1}
            disabled={ds || !!this.props.teachingState}
          />
          {mode === 2 ? (
            <span style={{ marginLeft: "24px", fontSize: "14px" }}>
              示教：
              <Switch
                checkedChildren="启动"
                unCheckedChildren="终止"
                checked={!!this.props.teachingState}
                className={`${prefixCls}-switch`}
                onClick={this.handleClick2}
                disabled={
                  ds || !!this.props.isMvc || !this.props.isForceControl
                }
              />
            </span>
          ) : (
            ""
          )}
        </form>

        <div
          style={{
            display: "inline-block",
            marginTop: "14px",
            paddingTop: "40px",
            paddingLeft: "70px",
          }}
        >
          {record}
          <Modal
            title="添加点确认"
            visible={this.state.visible1}
            onOk={this.addPointOk}
            onCancel={this.addPointCancel}
            centered={true}
            cancelText="取消"
            okText="确定"
          >
            <p>是否添加一个mvj/mvl/mvc点？</p>
          </Modal>
          <Modal
            title="删除点确认"
            visible={this.state.visible2}
            onOk={this.removePointOk}
            onCancel={this.removePointCancel}
            centered={true}
            cancelText="取消"
            okText="确定"
          >
            <p>是否删除上个添加的点？</p>
          </Modal>
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

// 组件用到的属性 //
function mapStateToProps(state) {
  return {
    teachingMode: state.server.teachingMode,
    isForceControl: state.server.isForceControl,
    teachingState: state.server.teachingState,
    isMvc: state.server.isMvc,
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
const RecordingMode = connect(mapStateToProps, mapDispatchToProps)(Recording);

RecordingMode.NAME = "力控模式选择";

export default RecordingMode;
