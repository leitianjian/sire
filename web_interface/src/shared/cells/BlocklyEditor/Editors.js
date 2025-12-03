import React, { Component, Fragment } from "react";
import { connect } from "react-redux";
import Button from "antd/lib/button";
import {
  DoubleRightOutlined,
  DoubleLeftOutlined,
  CaretRightOutlined,
  PauseOutlined,
  StopOutlined,
  StepForwardOutlined,
} from "@ant-design/icons";
import Tooltip from "antd/lib/tooltip";

import BlocklyWorkspace from "./BlocklyWorkspace";

import Blockly, { Xml } from "blockly";
import { ResizableBox } from "react-resizable";

import { withSize } from "react-sizeme";
import Modal from "antd/lib/modal";

import { IntlProvider, FormattedMessage } from "react-intl";

import { setLoadingBlocks, setNotLoadingBlocks } from "./blocks";
import "./robot";
import "./generators";
import {
  VARIABLES_TOOLBOX_CATEGORIES,
  PROGRAM_TOOLBOX_CATEGORIES,
} from "./initContent";
import { CreateFileModal } from "./modals";

import {
  createProgram,
  deleteProgram,
  updateProgram,
  renameProgram,
  sendCmd,
  actCopyBlock,
} from "../../../state/actions";

import {
  getStateCode,
  getCurrentToolId,
  getCurrentWobjId,
  getProgramLine,
  getProErrCode,
  getProErrMsg,
  getMemoizeCsStarted,
  getWebsocketConnected,
} from "../../../state/selectors";

class Editors extends Component {
  state = {
    creatingFile: false,
    hasSaved: true,
    shouldSaveModalVisible: false,
    shouldSaveProgram: null,
    shouldSaveFilename: null,
    shouldDeleteFile: false,
  };

  gotoIndex = "0";

  // for goto and save status //
  changeListener = (event) => {
    if (event.workspaceId === this.programWorkspace.id) {
      const blk = this.programWorkspace.getBlockById(event.blockId);
      if (!!blk && !!blk.data) this.gotoIndex = blk.data;
    }
  };

  workspaceDidChange = (workspace) => {
    this.SaveProgram(this.props.program, this.props.fileName);
  };

  mountProgram = () => {
    // 定时保存 //
    console.log("mount program");
    console.log(this.props.program);
    // if(!!this.interval)clearInterval(this.interval);
    if (!!this.programWorkspace && !!this.variablesWorkspace) {
      console.log("mount program - check workspace");
      // for auto drop list //
      // this.variablesWorkspace.fileName = this.props.fileName;
      // this.programWorkspace.fileName = this.props.fileName;
      // this.variablesWorkspace.programWorkspace = this.programWorkspace;
      // this.programWorkspace.variablesWorkspace = this.variablesWorkspace;

      if (!!this.props.program && !!this.props.fileName) {
        console.log("mount program - check workspace - check file name");
        // 更新workspace中变量列表 //
        this.programWorkspace.robtarget_variable = [];
        this.programWorkspace.speed_variable = [];
        this.programWorkspace.jointtarget_variable = [];
        this.programWorkspace.zone_variable = [];
        this.programWorkspace.function = [];

        this.props.program.files.forEach((f) => {
          const fileName = f.name.slice(0, f.name.length - 4);
          if (fileName !== this.props.fileName) {
            if (f.robtargets) {
              f.robtargets.forEach((t) => {
                this.programWorkspace.robtarget_variable.push([
                  fileName + "." + t,
                  fileName + "." + t,
                ]);
              });
            }
            if (f.jointtargets) {
              f.jointtargets.forEach((t) => {
                this.programWorkspace.jointtarget_variable.push([
                  fileName + "." + t,
                  fileName + "." + t,
                ]);
              });
            }
            if (f.speeds) {
              f.speeds.forEach((t) => {
                this.programWorkspace.speed_variable.push([
                  fileName + "." + t,
                  fileName + "." + t,
                ]);
              });
            }
            if (f.zones) {
              f.zones.forEach((t) => {
                this.programWorkspace.zone_variable.push([
                  fileName + "." + t,
                  fileName + "." + t,
                ]);
              });
            }
            if (f.functions) {
              f.functions.forEach((t) => {
                this.programWorkspace.function.push([
                  fileName + "." + t,
                  fileName + "." + t,
                ]);
              });
            }
          }
        });

        // 更新程序块 //
        let proXml = "";
        let datXml = "";

        this.props.program.files.forEach((f) => {
          if (f.name === this.props.fileName + ".pro") {
            proXml = f.content;
          } else if (f.name === this.props.fileName + ".dat") {
            datXml = f.content;
          }
        });

        // 加载程序 //
        console.log("test", proXml, datXml);
        if (!!proXml && !!datXml) {
          setLoadingBlocks();

          let dom = Xml.textToDom(datXml);
          this.variablesWorkspace.clear();
          Blockly.Xml.clearWorkspaceAndLoadFromXml(
            dom,
            this.variablesWorkspace
          );
          dom = Xml.textToDom(proXml);
          Blockly.Xml.clearWorkspaceAndLoadFromXml(dom, this.programWorkspace);

          setNotLoadingBlocks();
        } else {
          setLoadingBlocks();

          this.variablesWorkspace.clear();
          this.programWorkspace.clear();

          setNotLoadingBlocks();
        }

        // 自动保存 //
        // this.interval = setInterval(()=>{console.log("save...");this.SaveProgram(this.props.program, this.props.fileName)}, 3000);
      }

      this.programWorkspace.addChangeListener(this.changeListener);
    }
  };

  SaveProgram = (program, fileName) => {
    if (!program || !fileName) return;

    // generate code //
    this.variablesWorkspace.generateId = 0;
    const code1 = Blockly.Robot.workspaceToCode(this.variablesWorkspace);
    this.programWorkspace.generateId = this.variablesWorkspace.generateId + 1;
    const code2 = Blockly.Robot.workspaceToCode(this.programWorkspace);
    const code = code1 + "\n" + code2;

    // get xml and dom //
    const newProDom = Blockly.Xml.workspaceToDom(this.programWorkspace);
    const newDatDom = Blockly.Xml.workspaceToDom(this.variablesWorkspace);

    const newProXml = Blockly.Xml.domToText(newProDom);
    const newDatXml = Blockly.Xml.domToText(newDatDom);

    // save files //
    const newProgram = { ...program };
    const datIdx = newProgram.files.findIndex((f) => {
      return f.name === fileName + ".dat";
    });
    const proIdx = newProgram.files.findIndex((f) => {
      return f.name === fileName + ".pro";
    });
    const arisIdx = newProgram.files.findIndex((f) => {
      return f.name === fileName + ".aris";
    });
    if (datIdx === -1) {
      newProgram.files.push({ name: fileName + ".dat", content: newDatXml });
    } else {
      newProgram.files[datIdx] = {
        ...newProgram.files[datIdx],
        content: newDatXml,
      };
    }
    if (proIdx === -1) {
      newProgram.files.push({ name: fileName + ".pro", content: newProXml });
    } else {
      newProgram.files[proIdx] = {
        ...newProgram.files[proIdx],
        content: newProXml,
      };
    }
    if (arisIdx === -1) {
      newProgram.files.push({ name: fileName + ".aris", content: code });
    } else {
      newProgram.files[arisIdx] = {
        ...newProgram.files[arisIdx],
        content: code,
      };
    }

    // dispach action //
    this.props.updateProgram(newProgram.name, newProgram);
  };

  DeleteProgram = () => {
    if (!this.props.program || !this.props.fileName) return;

    const newProgram = { ...this.props.program };
    const datIdx = newProgram.files.findIndex((f) => {
      return f.name === this.props.fileName + ".dat";
    });
    if (datIdx !== -1) newProgram.files.splice(datIdx, 1);
    const proIdx = newProgram.files.findIndex((f) => {
      return f.name === this.props.fileName + ".pro";
    });
    if (proIdx !== -1) newProgram.files.splice(proIdx, 1);
    const arisIdx = newProgram.files.findIndex((f) => {
      return f.name === this.props.fileName + ".aris";
    });
    if (arisIdx !== -1) newProgram.files.splice(arisIdx, 1);

    this.props.updateProgram(newProgram.name, newProgram);
  };

  DownloadProgram = () => {
    // save current workspace //
    this.SaveProgram(this.props.program, this.props.fileName);

    const files = [];

    this.props.program.files.forEach((f) => {
      if (f.name.slice(-4) === "aris") files.push(f);
    });

    this.props.sendCmd(
      `program --content={${JSON.stringify(files)}}`,
      this.props.cell.i
    );
  };

  startProgram = () => {
    this.props.sendCmd("program --start", this.props.cell.i);
  };

  stopProgram = () => {
    this.props.sendCmd("program --stop", this.props.cell.i);
  };

  // componentDidMount() {
  //   this.mountProgram();
  // }

  componentDidUpdate(prevProps, prevState, snapshot) {
    if (
      !!this.variablesWorkspace &&
      !!this.programWorkspace &&
      (prevProps.program.name !== this.props.program.name ||
        prevProps.fileName !== this.props.fileName)
    ) {
      if (this.state.hasSaved) {
        // this.mountProgram();
      } else {
        this.setState({
          shouldSaveModalVisible: true,
          shouldSaveProgram: prevProps.program,
          shouldSaveFilename: prevProps.fileName,
        });
      }
    }

    // 高亮，highlight //
    if (this.programWorkspace) {
      if (this.props.stateCode !== 100 && this.props.stateCode !== 200) {
        const blk = this.programWorkspace.getAllBlocks().find((blk) => {
          return blk.data === this.props.line;
        });
        if (blk) {
          this.programWorkspace.scroll(
            this.programWorkspace.getWidth() / 8 -
            blk.getRelativeToSurfaceXY().x,
            80 - blk.getRelativeToSurfaceXY().y
          );
          this.programWorkspace.highlightBlock(blk ? blk.id : null);
        }
      } else {
        this.programWorkspace.highlightBlock(null);
      }
    }
  }

  shouldComponentUpdate(nextProps, nextState) {
    if (
      nextProps.size.width !== this.props.size.width ||
      nextProps.size.height !== this.props.size.height) {
      Blockly.svgResize(this.programWorkspace);
      Blockly.svgResize(this.variablesWorkspace);
    }
    return true;
  }
  render() {
    console.log("render blockly editor ");
    // state code
    // 100: "去使能",
    // 200: "手动",
    // 300: "自动",
    // 400: "程序运行中",
    // 410: "程序暂停中",
    // 420: "程序停止",
    // 500: "错误"

    const {
      prefixCls,
      size: { width, height },
      options,
      sendCmd,
      stateCode,
      proErrCode,
      proErrMsg,
    } = this.props;

    const isRunning = stateCode === 400;
    const runState =
      stateCode === 300 ||
      stateCode === 410 ||
      stateCode === 420 ||
      stateCode === 500;
    const ds = !this.props.isWsConnected || !this.props.isCsStarted;

    // if (this.programWorkspace) {
    //   this.programWorkspace.options.hasCss = false;
    // }

    // make readonly when running //
    if (!!this.programWorkspace && !!this.variablesWorkspace) {
      const readonly =
        stateCode === 300 ||
        stateCode === 400 ||
        stateCode === 410 ||
        stateCode === 420 ||
        stateCode === 500;
      if (readonly) {
        this.programWorkspace.getAllBlocks().forEach((d) => {
          d.setDeletable(false);
          d.setEditable(false);
          d.setMovable(false);
        });

        this.variablesWorkspace.getAllBlocks().forEach((d) => {
          d.setDeletable(false);
          d.setEditable(false);
          d.setMovable(false);
        });
      } else {
        this.programWorkspace.getAllBlocks().forEach((d) => {
          d.setDeletable(true);
          d.setEditable(true);
          d.setMovable(true);
        });

        this.variablesWorkspace.getAllBlocks().forEach((d) => {
          d.setDeletable(true);
          d.setEditable(true);
          d.setMovable(true);
        });
      }
    }

    const workspaceConfiguration = {
      // css: false,
      grid: {
        spacing: 20,
        length: 3,
        colour: "#ccc",
        snap: true,
      },
      move: {
        scrollbars: true,
        drag: true,
        wheel: true,
      },
      zoom: {
        controls: true,
        wheel: true,
        startScale: 1.0,
        maxScale: 3,
        minScale: 0.3,
        scaleSpeed: 1.2,
        pinch: true,
      },
    };

    let variablesHeight = options.variablesHeight
      ? parseFloat(options.variablesHeight)
      : 200;
    if (variablesHeight <= 50) {
      variablesHeight = 50;
    }
    const dividerHeight = 68;
    const programHeight = height - variablesHeight - dividerHeight;

    let initialVariableXml = "";
    let initialProgramXml = "";
    this.props.program.files.forEach((f) => {
      if (f.name === this.props.fileName + ".pro") {
        initialProgramXml = f.content;
      } else if (f.name === this.props.fileName + ".dat") {
        initialVariableXml = f.content;
      }
    });
    return (
      <IntlProvider>
        <Fragment>
          <div className={`${prefixCls}-editors`}>
            {proErrCode && proErrCode !== 0 ? (
              <span style={{ color: "red", display: "block" }}>
                错误信息：{proErrMsg}
              </span>
            ) : (
              <span style={{ color: "#52c41a", display: "block" }}>
                return_code：{proErrCode}
              </span>
            )}
            {!!this.props.program &&
              !!this.props.fileName &&
              !options.hideVariablesWorkspace && (
                <ResizableBox
                  width={width}
                  height={variablesHeight}
                  minConstraints={[width, 100]}
                  maxConstraints={[width, height - 30 - 100]}
                  className={`${prefixCls}-editors-variables`}
                  axis={"y"}
                  handle={<div className={`${prefixCls}-divider`}></div>}
                  onResize={(e, { size: { height, width } }) => {
                    this.props.updateOptions({
                      ...options,
                      variablesHeight: height,
                    });
                    Blockly.svgResize(this.programWorkspace);
                    Blockly.svgResize(this.variablesWorkspace);
                  }}
                >
                  <div
                    className={`${prefixCls}-variable-scroll`}
                    style={{ height: variablesHeight - 5 }}
                  >
                    <BlocklyWorkspace
                      workspaceHandle={(workspace) => {
                        this.variablesWorkspace = workspace;
                      }}
                      initialXml={initialVariableXml}
                      toolboxConfiguration={VARIABLES_TOOLBOX_CATEGORIES}
                      workspaceConfiguration={workspaceConfiguration}
                      className={`${prefixCls}-full-height`}
                      onWorkspaceChange={this.workspaceDidChange}
                    />
                  </div>
                </ResizableBox>
              )}
            {!!this.props.program && !!this.props.fileName && (
              <div
                className={`${prefixCls}-editors-program ${options.hideVariablesWorkspace
                    ? `${prefixCls}-editors-program-full-height`
                    : ""
                  }`}
                style={{
                  height: programHeight,
                }}
              >
                <div
                  className={`${prefixCls}-editors-scroll`}
                  style={{ height: programHeight }}
                >
                  <BlocklyWorkspace
                    workspaceHandle={(workspace) => {
                      this.programWorkspace = workspace;
                    }}
                    initialXml={initialProgramXml}
                    toolboxConfiguration={PROGRAM_TOOLBOX_CATEGORIES}
                    workspaceConfiguration={workspaceConfiguration}
                    className={`${prefixCls}-full-height`}
                    onWorkspaceChange={this.workspaceDidChange}
                  />
                  <Tooltip
                    placement="bottom"
                    title={
                      options.hideVariablesWorkspace ? (
                        <FormattedMessage id="显示变量定义" />
                      ) : (
                        <FormattedMessage id="隐藏变量定义" />
                      )
                    }
                  >
                    <DoubleRightOutlined
                      className={`${prefixCls}-editors-program-toggle`}
                      onClick={() => {
                        this.props.updateOptions({
                          ...options,
                          hideVariablesWorkspace:
                            !options.hideVariablesWorkspace,
                        });
                      }}
                    />
                  </Tooltip>
                </div>
              </div>
            )}
          </div>
          <div className={`${prefixCls}-operations`}>
            <div
              className={`${prefixCls}-operations-left`}
              style={{ pointerEvents: "none" }}
            >
              {!!options.hideSidebar && (
                <DoubleRightOutlined
                  className={`${prefixCls}-sidebar-toggle`}
                  onClick={() => {
                    this.props.updateOptions({ ...options, hideSidebar: "" });
                  }}
                />
              )}
              {!options.hideSidebar && (
                <DoubleLeftOutlined
                  className={`${prefixCls}-sidebar-toggle`}
                  onClick={() => {
                    this.props.updateOptions({
                      ...options,
                      hideSidebar: "true",
                    });
                  }}
                />
              )}
              <span className={`${prefixCls}-operations-left-name`}>
                {this.props.program.name}
              </span>
            </div>
            <CreateFileModal
              program={this.props.program}
              visible={this.state.creatingFile}
              closeModal={() => {
                this.setState({ creatingFile: false });
              }}
            />
            <Button
              size={"middle"}
              type={"primary"}
              disabled={ds || isRunning}
              style={{ pointerEvents: "auto" }}
              onClick={() => {
                this.setState({ creatingFile: true });
              }}
            >
              <FormattedMessage id="新建" />
            </Button>
            <Button
              size={"middle"}
              type={"primary"}
              disabled={ds || isRunning}
              style={{ pointerEvents: "auto" }}
              onClick={() => {
                this.setState({ shouldDeleteFile: true });
              }}
            >
              <FormattedMessage id="删除" />
            </Button>
            <Modal
              // wrapClassName={`${prefixCls}-create-program-modal`}
              title={<FormattedMessage id="确定删除？" />}
              okText={<FormattedMessage id="是" />}
              cancelText={<FormattedMessage id="否" />}
              visible={this.state.shouldDeleteFile}
              onOk={() => {
                this.DeleteProgram();
                this.setState({ fileName: null, shouldDeleteFile: false });
              }}
              onCancel={() => {
                this.setState({ fileName: null, shouldDeleteFile: false });
              }}
            />
            <Button
              size={"middle"}
              type={"primary"}
              disabled={ds || this.state.hasSaved || isRunning}
              style={{ pointerEvents: "auto" }}
              onClick={() =>
                this.SaveProgram(this.props.program, this.props.fileName)
              }
            >
              <FormattedMessage id="保存" />
            </Button>
            <Modal
              // wrapClassName={`${prefixCls}-create-program-modal`}
              title={<FormattedMessage id="程序未保存，是否保存？" />}
              okText={<FormattedMessage id="是" />}
              cancelText={<FormattedMessage id="否" />}
              visible={this.state.shouldSaveModalVisible}
              onOk={() => {
                this.SaveProgram(
                  this.state.shouldSaveProgram,
                  this.state.shouldSaveFilename
                );
                this.setState({ shouldSaveModalVisible: false });
                this.mountProgram();
              }}
              onCancel={() => {
                this.setState({ shouldSaveModalVisible: false });
                this.mountProgram();
              }}
            />
            <Button
              size={"middle"}
              type={"primary"}
              disabled={ds || isRunning}
              style={{ pointerEvents: "auto" }}
              onClick={this.DownloadProgram}
            >
              <FormattedMessage id="下载UI程序" />
            </Button>
            <Button
              size={"middle"}
              type={"primary"}
              disabled={ds || isRunning}
              style={{ pointerEvents: "auto" }}
              onClick={() => {
                this.props.sendCmd("program --goto_main");
              }}
            >
              <FormattedMessage id="跳转到main函数" />
            </Button>
            <Button
              size={"middle"}
              type={"primary"}
              disabled={!runState}
              onClick={this.startProgram}
              style={{ pointerEvents: "auto" }}
            >
              <CaretRightOutlined />
            </Button>
            <Button
              size={"middle"}
              type={"primary"}
              disabled={ds || !isRunning || !!(stateCode === 410)}
              style={{ pointerEvents: "auto" }}
              onClick={() => {
                this.props.sendCmd("program --pause");
              }}
            >
              <PauseOutlined />
            </Button>
            <Button
              size={"middle"}
              type={"primary"}
              disabled={
                ds || (!isRunning && stateCode !== 410) || stateCode === 420
              }
              style={{ pointerEvents: "auto" }}
              onClick={() => {
                this.stopProgram();
              }}
            >
              <StopOutlined style={{ color: "red" }} />
            </Button>
            <Button
              size={"middle"}
              type={"primary"}
              disabled={ds || !runState}
              style={{ pointerEvents: "auto" }}
              onClick={(e) => {
                this.props.sendCmd("program --forward");
              }}
            >
              <StepForwardOutlined />
            </Button>
            <Button
              size={"middle"}
              type={"primary"}
              style={{ pointerEvents: "auto" }}
              onClick={(e) => {
                if (!!this.props.fileName && this.gotoIndex) {
                  this.props.sendCmd(
                    `program --goto=${this.props.fileName}.${this.gotoIndex}`
                  );
                }
              }}
            >
              goto
            </Button>
            <Button
              size={"middle"}
              type={"primary"}
              disabled={ds}
              style={{ pointerEvents: "auto" }}
              onClick={(e) => {
                sendCmd("program --clear_error");
              }}
            >
              <FormattedMessage id="清除程序错误" />
            </Button>
          </div>
        </Fragment>
      </IntlProvider>
    );
  }
}

function mapStateToProps(state) {
  return {
    line: getProgramLine(state),
    stateCode: getStateCode(state),
    proErrCode: getProErrCode(state),
    proErrMsg: getProErrMsg(state),
    toolId: getCurrentToolId(state),
    wobjId: getCurrentWobjId(state),
    isCsStarted: getMemoizeCsStarted(state),
    isWsConnected: getWebsocketConnected(state),
  };
}

function mapDispatchToProps(dispatch) {
  return {
    createProgram: (...args) => dispatch(createProgram(...args)),
    deleteProgram: (...args) => dispatch(deleteProgram(...args)),
    updateProgram: (...args) => dispatch(updateProgram(...args)),
    renameProgram: (...args) => dispatch(renameProgram(...args)),
    sendCmd: (...args) => dispatch(sendCmd(...args)),
    dspCopyBlock: (...args) => dispatch(actCopyBlock(...args)),
  };
}

export default connect(
  mapStateToProps,
  mapDispatchToProps
)(withSize({ monitorHeight: true, refreshRate: 30 })(Editors));
