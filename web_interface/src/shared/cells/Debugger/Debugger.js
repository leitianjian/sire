import React from "react";
import Row from "antd/lib/row";
import Button from "antd/lib/button";
import Input from "antd/lib/input";
import CellBase from "../CellBase";
import { connect } from "react-redux";
import Terminal from "terminal-in-react";
import { getlocalestate } from "../../../state/selectors";
import { createSelector } from "reselect";

import { sendCmd } from "../../../state/actions";

// 进行中英文切换
import zhCN from "../locale/zhCN";
import { IntlProvider } from "react-intl";
import enUS from "../locale/enUS";
import store from "../../../index";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

const { TextArea } = Input;

class Debugger extends CellBase {
  interval = null;
  timeout = null;
  state = {
    result: null,
    loading: "false",
    disabled: false,
    time: 3,
  };

  getSendCmdFn = () => {
    const { sendCmd } = this.props;
    const options = this.getOptions();
    if (options.command) {
      options.command.split("\n").forEach((cmd) => {
        cmd = cmd.trim();
        if (cmd) {
          sendCmd(cmd, (msg) => {
            this.setState({ result: msg.jsData });
          });
          this.terminal.runCommandOnActive("showCmd " + cmd);
        }
      });
    }
  };

  getSendCmdFn_muti = () => {
    return () => {
      const { sendCmd } = this.props;
      const options = this.getOptions();
      if (options.command) {
        const command = options.command.trim();
        sendCmd(command, (msg) => {
          this.setState({ result: msg.jsData });
        });
        this.terminal.runCommandOnActive("showCmd " + command);
      }
    };
  };

  getOnMouseDownFn = () => {
    return () => {
      if (this.interval) {
        clearInterval(this.interval);
      }
      const fn = this.getSendCmdFn();
      const self = this;
      const { commandSendInterval, commandSendDelay } = this.props;
      self.timeout = setTimeout(() => {
        self.interval = setInterval(fn, commandSendInterval);
      }, commandSendDelay);
    };
  };

  getOnMouseUpFn = () => {
    return () => {
      if (this.interval) {
        clearInterval(this.interval);
      }
      if (this.timeout) {
        clearTimeout(this.timeout);
      }
    };
  };

  // shouldComponentUpdate(nextProps, nextState, nextContext) {
  // if(nextProps.result !== this.props.result){
  // this.result = nextProps.result
  // this.terminal.runCommandOnActive("showResult")
  // }
  // if(nextProps.cell !== this.props.cell || nextProps.result !== this.props.result){
  // return true
  // }
  // return false
  // }

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, localestate } = this.props;

    const options = this.getOptions();
    const prefixCls = getPrefixCls("debugger", customizePrefixCls);
    const self = this;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <div className={`${prefixCls}`}>
          <Row>
            {localestate === "chinese" ? (
              <TextArea
                rows={2}
                value={options.command}
                onChange={(e) => {
                  this.updateOptions({ command: e.target.value });
                }}
                placeholder="请输入一行机器人指令"
              />
            ) : (
              <TextArea
                rows={2}
                value={options.command}
                onChange={(e) => {
                  this.updateOptions({ command: e.target.value });
                }}
                placeholder="Please enter a line of robot commands!"
              />
            )}

            <Button
              onClick={this.getSendCmdFn}
              disabled={this.state.disabled}
              type="primary"
              shape="round"
              icon="play-circle"
              size="large"
            >
              Send
            </Button>
            <Button
              onClick={this.getSendCmdFn_muti()}
              type="primary"
              shape="round"
              icon="play-circle"
              size="large"
              style={{ display: "relative", left: "10px" }}
            >
              Send-Muti-Row
            </Button>
            <Button
              onClick={() => {
                console.log(store.getState());
              }}
              type="primary"
              shape="round"
              icon="play-circle"
              size="large"
              style={{ display: "relative", left: "10px" }}
            >
              Console Status
            </Button>
            <div className={`${prefixCls}-terminal`}>
              <Terminal
                color="green"
                backgroundColor="black"
                barColor="black"
                hideTopBar={true}
                allowTabs={false}
                commands={{
                  showCmd: (args, print, runCommand) => {
                    if (args[1]) {
                      print(args.slice(1).join(" "));
                    } else {
                      const options = self.getOptions();
                      if (options.command) {
                        print(options.command);
                      }
                    }
                  },
                  send: (args, print, runCommand) => {
                    self.props.sendCmd(args[1], (msg) => {
                      this.setState({ result: msg.jsData });
                    });
                  },
                  showResult: (args, print, runCommand) => {
                    print(JSON.stringify(this.state.result));
                  },
                }}
                descriptions={{
                  send: '发送指令给机器人, "send [机器人指令]", 例如 "send get"',
                  showResult: "显示最近一条返回值",
                  showCmd: "显示当前指令值",
                }}
                ref={(ref) => (this.terminal = ref)}
              />
            </div>
          </Row>
        </div>
      </IntlProvider>
    );
  };
}

Debugger.propTypes = {
  // results: PropTypes.array.isRequired
};



function mapStateToProps(state, props) {
  return {
    localestate: getlocalestate(state),
  };
}

function mapDispatchToProps(dispatch) {
  return {
    sendCmd: (...args) => dispatch(sendCmd(...args)),
  };
}

// const getState = createSelector(getlocalestate, (localestate) => {
//   return {
//     localestate,
//   };
// });

// const makeMapStateToProps = () => {
//   // const getResultsByCell = makeGetResultsByCell(10)
//   const mapStateToProps = (state, props) => {
//     return {
//       localestate: getState(state, props).localestate,
//       // results: getResultsByCell(state, props),
//       // result: getResultByChannel(state, props.cell.i),
//     };
//   };
//   return mapStateToProps;
// };

const ConnectedDebugger = connect(mapStateToProps, mapDispatchToProps)(Debugger);

ConnectedDebugger.NAME = "调试面板";

export default ConnectedDebugger;
