import React from "react";
import { IntlProvider } from "react-intl";
import { connect } from "react-redux";
import PropTypes from "prop-types";
import { ConfigConsumer } from "antd/lib/config-provider";
import {
  CaretLeftOutlined,
  CaretRightOutlined,
  WifiOutlined,
  CheckCircleFilled,
} from "@ant-design/icons";
import Row from "antd/lib/row";
import Button from "antd/lib/button";
import Slider from "antd/lib/slider";

import CellBase from "../CellBase";
import {
  getMemoizeMotionPos,
  getMemoizeMotionState,
  getMemoizeSlaveAlState,
  getMemoizeSlaveOnlineState,
  getMemoizeEndPE,
  getMemoizeCsStarted,
  getWebsocketConnected,
  getStateCode,
} from "../../../state/selectors";
import { sendCmd } from "../../../state/actions";

class JogJoint extends CellBase {
  interval = null;
  timeout = null;

  getOnMouseDownFn = (cmd) => {
    return () => {
      if (this.interval) {
        clearInterval(this.interval);
      }
      const { commandSendInterval, commandSendDelay } = this.props;

      const fn = () => {
        this.props.sendCmd(cmd);
      };
      this.timeout = setTimeout(() => {
        this.interval = setInterval(fn, commandSendInterval);
      }, commandSendDelay);
    };
  };

  getOnMouseUpFn = (cmd) => {
    return () => {
      if (this.timeout) clearTimeout(this.timeout);
      if (this.interval) clearInterval(this.interval);
      if (this.timeout && this.interval && cmd) {
        setTimeout(() => {
          this.props.sendCmd(cmd);
        }, 10);
        this.timeout = null;
        this.interval = null;
      }
    };
  };

  renderHeader = (prefixCls) => {
    const options = this.getOptions();
    const infos = { 0: "轴空间", 1: "工件坐标系", 2: "工具坐标系" };

    return (
      <div>
        <div className={`${prefixCls}`}>
          <Button
            icon={<CaretLeftOutlined />}
            type="primary"
            className={`${prefixCls}-header-btn`}
            onClick={() => {
              this.updateOptions({ ...options, mode: (options.mode + 2) % 3 });
            }}
          />
          <span
            style={{ display: "inline-block", width: "84px", opacity: 0.75 }}
          >
            {`${options.mode !== undefined ? infos[options.mode] : ""}`}
          </span>
          <Button
            icon={<CaretRightOutlined />}
            type="primary"
            className={`${prefixCls}-header-btn`}
            onClick={() => {
              this.updateOptions({ ...options, mode: (options.mode + 1) % 3 });
            }}
          />
        </div>
        <div style={{ height: "32px" }} />
      </div>
    );
  };

  renderJogCoordinate = (prefixCls) => {
    const { endPE, motionVelPercent, toolId, wobjId } = this.props;
    const names = ["X", "Y", "Z", "RX", "RY", "RZ"];
    const types = ["x", "y", "z", "rx", "ry", "rz"];

    const options = this.getOptions();
    const cor = options.mode === 1 ? 0 : 1;

    const ds =
      !this.props.isWsConnected ||
      !this.props.isCsStarted ||
      this.props.stateCode !== 200;

    return (
      <div className={`${prefixCls}`}>
        {names.map((name, index) => (
          <div key={name} style={{ height: "50px" }}>
            <Button
              disabled={ds}
              type={"primary"}
              size={"large"}
              className={`${prefixCls}-jog-btn`}
              onClick={() => {
                this.getSendCmdFn(
                  `j${types[index]} --vel_percent=${motionVelPercent} --direction=-1 --cor=${cor} --tool=tool${toolId} --wobj=wobj${wobjId}`
                )();
              }}
              onMouseDown={() => {
                this.getOnMouseDownFn(
                  `j${types[index]} --vel_percent=${motionVelPercent} --direction=-1 --cor=${cor} --tool=tool${toolId} --wobj=wobj${wobjId}`
                )();
              }}
              onMouseUp={() => {
                this.getOnMouseUpFn(`j${types[index]} --stop`)();
              }}
              onMouseOut={() => {
                this.getOnMouseUpFn(`j${types[index]} --stop`)();
              }}
              onTouchStart={() => {
                this.getOnMouseDownFn(
                  `j${types[index]} --vel_percent=${motionVelPercent} --direction=-1 --cor=${cor} --tool=tool${toolId} --wobj=wobj${wobjId}`
                )();
              }}
              onTouchEnd={() => {
                this.getOnMouseUpFn(`j${types[index]} --stop`)();
              }}
              onTouchCancel={() => {
                this.getOnMouseUpFn(`j${types[index]} --stop`)();
              }}
            >
              {name}-
            </Button>
            <span
              style={{ display: "inline-block", width: "60px", opacity: 0.75 }}
            >
              {index < 3 && (
                <span>
                  {!!endPE && (!!endPE[index] || endPE[index] === 0)
                    ? (endPE[index] * 1000).toFixed(3)
                    : "null"}
                </span>
              )}
              {index >= 3 && (
                <span>
                  {!!endPE && (!!endPE[index] || endPE[index] === 0)
                    ? ((endPE[index] / Math.PI) * 180).toFixed(3)
                    : "null"}
                </span>
              )}
            </span>
            <Button
              disabled={ds}
              type={"primary"}
              size={"large"}
              className={`${prefixCls}-jog-btn`}
              onClick={() => {
                this.getSendCmdFn(
                  `j${types[index]} --vel_percent=${motionVelPercent} --direction=1 --cor=${cor} --tool=tool${toolId} --wobj=wobj${wobjId}`
                )();
              }}
              onMouseDown={() => {
                this.getOnMouseDownFn(
                  `j${types[index]} --vel_percent=${motionVelPercent} --direction=1 --cor=${cor} --tool=tool${toolId} --wobj=wobj${wobjId}`
                )();
              }}
              onMouseUp={() => {
                this.getOnMouseUpFn(`j${types[index]} --stop`)();
              }}
              onMouseOut={() => {
                this.getOnMouseUpFn(`j${types[index]} --stop`)();
              }}
              onTouchStart={() => {
                this.getOnMouseDownFn(
                  `j${types[index]} --vel_percent=${motionVelPercent} --direction=1 --cor=${cor} --tool=tool${toolId} --wobj=wobj${wobjId}`
                )();
              }}
              onTouchEnd={() => {
                this.getOnMouseUpFn(`j${types[index]} --stop`)();
              }}
              onTouchCancel={() => {
                this.getOnMouseUpFn(`j${types[index]} --stop`)();
              }}
            >
              {name}+
            </Button>
          </div>
        ))}
      </div>
    );
  };

  renderJogJoint = (prefixCls) => {
    const {
      slaveOnlineState,
      slaveAlState,
      motionState,
      motionPos,
      motionVelPercent,
    } = this.props;

    const posMax = [145, 60, 55, 122, 135, 360];
    const posMin = [-145, -60, -55, -122, -135, -360];
    const ds =
      !this.props.isWsConnected ||
      !this.props.isCsStarted ||
      this.props.stateCode !== 200;

    return (
      <div className={`${prefixCls}`}>
        {[0, 1, 2, 3, 4, 5].map((i) => (
          <div key={i} style={{ height: "50px" }}>
            <Button
              disabled={ds}
              type="primary"
              size={"large"}
              className={`${prefixCls}-jog-btn`}
              onClick={() => {
                this.props.sendCmd(
                  `j${i + 1} --direction=-1 --vel_percent=${motionVelPercent}`
                );
              }}
              onMouseDown={() => {
                this.getOnMouseDownFn(
                  `j${i + 1} --direction=-1 --vel_percent=${motionVelPercent}`
                )();
              }}
              onMouseUp={() => {
                this.getOnMouseUpFn(`j${i + 1} --stop`)();
              }}
              onMouseOut={() => {
                this.getOnMouseUpFn(`j${i + 1} --stop`)();
              }}
              onTouchStart={() => {
                this.getOnMouseDownFn(
                  `j${i + 1} --direction=-1 --vel_percent=${motionVelPercent}`
                )();
              }}
              onTouchEnd={() => {
                this.getOnMouseUpFn(`j${i + 1} --stop`)();
              }}
              onTouchCancel={() => {
                this.getOnMouseUpFn(`j${i + 1} --stop`)();
              }}
            >
              J{i + 1}-
            </Button>
            <div className={`${prefixCls}-status`}>
              {!!motionPos && (!!motionPos[i] || motionPos[i] === 0) ? (
                <span>{((motionPos[i] / Math.PI) * 180).toFixed(3)}</span>
              ) : (
                <span>null</span>
              )}
              {!!motionPos &&
                (!!motionPos[i] || motionPos[i] === 0) &&
                !(
                  ((motionPos[i] / Math.PI) * 180).toFixed(3) <= posMin[i] ||
                  ((motionPos[i] / Math.PI) * 180).toFixed(3) >= posMax[i]
                ) && (
                  <Slider
                    defaultValue={((motionPos[i] / Math.PI) * 180).toFixed(3)}
                    value={((motionPos[i] / Math.PI) * 180).toFixed(3)}
                    style={{ margin: "0em" }}
                    min={posMin[i]}
                    max={posMax[i]}
                    disabled="true"
                  />
                )}
              {!!motionPos &&
                (!!motionPos[i] || motionPos[i] === 0) &&
                (((motionPos[i] / Math.PI) * 180).toFixed(3) <= posMin[i] ||
                  ((motionPos[i] / Math.PI) * 180).toFixed(3) >= posMax[i]) && (
                  <Slider
                    className={`${prefixCls}-slider`}
                    defaultValue={((motionPos[i] / Math.PI) * 180).toFixed(3)}
                    style={{ margin: "0em" }}
                    min={posMin[i]}
                    max={posMax[i]}
                    disabled="true"
                  />
                )}
              <div className={`${prefixCls}-status-icons`}>
                <WifiOutlined
                  className={`${
                    !slaveOnlineState || !slaveOnlineState[i]
                      ? `${prefixCls}-slave-online-state disabled`
                      : `${prefixCls}-slave-online-state ${prefixCls}-slave-online-state-${
                          slaveAlState && slaveAlState[i]
                        }`
                  }`}
                />
                {!!(motionState && motionState[i]) && (
                  <CheckCircleFilled className={"success"} />
                )}
                {!!(!motionState || !motionState[i]) && (
                  <CheckCircleFilled className={"disabled"} />
                )}
              </div>
            </div>
            <Button
              disabled={ds}
              type="primary"
              size={"large"}
              className={`${prefixCls}-jog-btn`}
              onClick={() => {
                this.props.sendCmd(
                  `j${i + 1} --direction=1 --vel_percent=${motionVelPercent}`
                );
              }}
              onMouseDown={() => {
                this.getOnMouseDownFn(
                  `j${i + 1} --direction=1 --vel_percent=${motionVelPercent}`
                )();
              }}
              onMouseUp={() => {
                this.getOnMouseUpFn(`j${i + 1} --stop`)();
              }}
              onMouseOut={() => {
                this.getOnMouseUpFn(`j${i + 1} --stop`)();
              }}
              onTouchStart={() => {
                this.getOnMouseDownFn(
                  `j${i + 1} --direction=1 --vel_percent=${motionVelPercent}`
                )();
              }}
              onTouchEnd={() => {
                this.getOnMouseUpFn(`j${i + 1} --stop`)();
              }}
              onTouchCancel={() => {
                this.getOnMouseUpFn(`j${i + 1} --stop`)();
              }}
            >
              J{i + 1}+
            </Button>
          </div>
        ))}
      </div>
    );
  };

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;

    const prefixCls = getPrefixCls("jog-joint", customizePrefixCls);

    const options = this.getOptions();
    let mode = options.mode;
    if (mode === undefined) {
      mode = 0;
      this.updateOptions({ ...options, mode });
    }

    return (
      <div>
        <Row>{this.renderHeader(prefixCls)}</Row>
        <Row>
          {mode === 0 && this.renderJogJoint(prefixCls)}
          {mode === 1 && this.renderJogCoordinate(prefixCls)}
          {mode === 2 && this.renderJogCoordinate(prefixCls)}
        </Row>
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

JogJoint.propTypes = {
  sendCmd: PropTypes.func.isRequired,
  vel: PropTypes.number,
  motionPos: PropTypes.array,
  slaveOnlineState: PropTypes.array,
};

function mapStateToProps(state) {
  return {
    motionVelPercent: state.robot.currentMotionVelPercent,
    coordinateId: state.robot.currentCoordinateId,
    toolId: state.robot.currentToolId,
    wobjId: state.robot.currentWobjId,
    endPE: getMemoizeEndPE(state),
    motionPos: getMemoizeMotionPos(state),
    motionState: getMemoizeMotionState(state),
    slaveOnlineState: getMemoizeSlaveOnlineState(state),
    slaveAlState: getMemoizeSlaveAlState(state),
    stateCode: getStateCode(state),
    isCsStarted: getMemoizeCsStarted(state),
    isWsConnected: getWebsocketConnected(state),
  };
}

function mapDispatchToProps(dispatch) {
  return {
    sendCmd: (...args) => dispatch(sendCmd(...args)),
  };
}

const ConnectedJogJoint = connect(
  mapStateToProps,
  mapDispatchToProps
)(JogJoint);

ConnectedJogJoint.NAME = "点动";

export default ConnectedJogJoint;
