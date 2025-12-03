import React, { Fragment } from "react";
import { connect } from "react-redux";
import Modal from "antd/lib/modal";
import Input from "antd/lib/input";
import CellBase from "../CellBase";

import Collapse from "antd/lib/collapse";
import Menu from "antd/lib/menu";
import "antd/lib/menu/style";
import {
  FileTextOutlined,
  PlusSquareFilled,
  EditFilled,
  CopyFilled,
  DeleteFilled,
} from "@ant-design/icons";

import "./blocks";
import "./robot";
import "./generators";
import Editors from "./Editors";
import { withSize } from "react-sizeme";
import { ResizableBox } from "react-resizable";
import Tooltip from "antd/lib/tooltip";
import ConnectedCreateProgramModal from "./CreateProgramModal";
import DeleteProgramModal from "./DeleteProgramModal";

import zhCN from "../locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "../locale/enUS";

import {
  loadPrograms,
  createProgram,
  toggleCreateProgramModal,
  deleteProgram,
  toggleDeleteProgramModal,
  toggleEditProgramName,
  renameProgram,
  resetRenameProgramStatus,
} from "../../../state/actions";

import { getStateCode, getProgramFile } from "../../../state/selectors";

const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

class Blockly extends CellBase {
  componentDidMount() {
    this.props.dspLoadPrograms();
  }

  renderPrograms(prefixCls) {
    return (
      <Fragment>
        <div className={`${prefixCls}-program-header`}>
          <FormattedMessage id="程序列表" />
          <Tooltip placement="top" title={<FormattedMessage id="新建程序" />}>
            <PlusSquareFilled
              className={`${prefixCls}-program-header-add`}
              onClick={this.props.dspToggleCreateProgramModal}
            />
          </Tooltip>
          <ConnectedCreateProgramModal
            createProgram={this.props.dspCreateProgram}
            toggleCreateProgramModal={this.props.dspToggleCreateProgramModal}
            showCreateProgramModal={this.props.showCreateProgramModal}
            creatingProgram={this.props.creatingProgram}
            creatingProgramFailure={this.props.creatingProgramFailure}
            creatingProgramSuccess={this.props.creatingProgramSuccess}
            prefixCls={prefixCls}
          />
        </div>
        {Object.keys(this.props.programs).length === 0 && (
          <div style={{ opacity: 0.5 }}>
            <FormattedMessage id="没有程序" />
          </div>
        )}
        {Object.keys(this.props.programs).length > 0 && (
          <div className={`${prefixCls}-program-collapse`}>
            <Collapse
              accordion
              activeKey={
                this.props.cell.options.selectedProgramName
                  ? this.props.cell.options.selectedProgramName
                  : null
              }
            >
              {Object.keys(this.props.programs)
                .sort()
                .map((name) => {
                  const program = this.props.programs[name];

                  // 将file_list的pro和dat文件统一 //
                  const fileList = [];
                  program.files.forEach((file) => {
                    const name = file.name.split(".")[0];
                    if (
                      fileList.find((value) => {
                        return value === name;
                      }) === undefined
                    ) {
                      fileList.push(name);
                    }
                  });

                  const header = (
                    <div
                      className={`${prefixCls}-program-name`}
                      onClick={() => {
                        if (
                          this.props.cell.options.selectedProgramName ===
                          program.name
                        ) {
                          return;
                        }

                        this.updateOptions({
                          ...this.props.cell.options,
                          selectedProgramName: program.name,
                          selectedFileName: null,
                        });
                      }}
                    >
                      {this.props.editingProgramName !== name && (
                        <span
                          className={`${prefixCls}-program-name-text`}
                          onDoubleClick={(e) => {
                            e.preventDefault();
                            e.stopPropagation();
                            this.props.dspToggleEditProgramName(name);
                          }}
                        >
                          {name}
                        </span>
                      )}
                      {this.props.editingProgramName === name && (
                        <Input
                          size={"small"}
                          onKeyUp={(e) => {
                            if (e.keyCode === 27) {
                              e.preventDefault();
                              e.stopPropagation();
                              this.props.dspToggleEditProgramName();
                            }
                          }}
                          onClick={(e) => {
                            e.preventDefault();
                            e.stopPropagation();
                          }}
                          className={`${prefixCls}-program-name-input`}
                          defaultValue={name}
                          onPressEnter={(e) => {
                            if (name !== e.target.value) {
                              this.props.dspRenameProgram(name, e.target.value);
                            } else {
                              this.props.dspToggleEditProgramName();
                            }
                          }}
                        />
                      )}
                      {this.props.editingProgramName !== name && (
                        <span className={`${prefixCls}-program-name-actions`}>
                          <EditFilled
                            onClick={(e) => {
                              e.preventDefault();
                              e.stopPropagation();
                              this.props.dspToggleEditProgramName(name);
                            }}
                          />
                          <CopyFilled
                            onClick={(e) => {
                              e.preventDefault();
                              e.stopPropagation();
                              let newName = name + " copy";
                              let getCopyName = false;
                              while (!getCopyName) {
                                getCopyName = true;
                                console.log("for each 2");
                                Object.keys(this.props.programs).forEach(
                                  (key) => {
                                    if (key === newName) {
                                      newName += " copy";
                                      getCopyName = false;
                                    }
                                  }
                                );
                              }
                              this.props.dspRenameProgram(name, newName, true);
                            }}
                          />
                          <DeleteFilled
                            onClick={(e) => {
                              e.preventDefault();
                              e.stopPropagation();
                              this.props.dspToggleDeleteProgramModal(name);
                            }}
                          />
                        </span>
                      )}
                    </div>
                  );

                  return (
                    <Collapse.Panel header={header} key={name}>
                      {!!program && (
                        <Menu
                          disable={this.props.stateCode === 400}
                          selectedKeys={[
                            this.props.stateCode === 400
                              ? this.props.currentFile.slice(
                                  0,
                                  this.props.currentFile.length - 5
                                )
                              : this.props.cell.options.selectedFileName,
                          ]}
                          onClick={(e) => {
                            if (
                              this.props.cell.options.selectedFileName === e.key
                            ) {
                              return;
                            }
                            this.updateOptions({
                              ...this.props.cell.options,
                              selectedFileName: `${e.key}`,
                            });
                          }}
                        >
                          {fileList.map((file, id) => {
                            return <Menu.Item key={file}>{file}</Menu.Item>;
                          })}
                        </Menu>
                      )}
                    </Collapse.Panel>
                  );
                })}
            </Collapse>
          </div>
        )}
        <DeleteProgramModal
          deleteProgram={this.props.dspDeleteProgram}
          toggleDeleteProgramModal={this.props.dspToggleDeleteProgramModal}
          showDeleteProgramModal={this.props.showDeleteProgramModal}
          deletingProgram={this.props.deletingProgram}
          deletingProgramFailure={this.props.deletingProgramFailure}
          deletingProgramSuccess={this.props.deletingProgramSuccess}
          prefixCls={prefixCls}
        />
        {!!this.props.editingProgramName &&
          !!this.props.renamingProgramFailure && (
            <Modal
              title={<FormattedMessage id="重命名错误" />}
              visible={
                !!this.props.editingProgramName &&
                !!this.props.renamingProgramFailure
              }
              onCancel={this.props.dspResetRenameProgramStatus}
              onOk={this.props.dspResetRenameProgramStatus}
              okText={<FormattedMessage id="确认" />}
              cancelButtonProps={{ style: { display: "none" } }}
            >
              <div className={"error"}>
                {!!this.props.renamingProgramFailure &&
                  this.props.renamingProgramFailure}
              </div>
            </Modal>
          )}
        {!this.props.editingProgramName && !!this.props.renamingProgramFailure && (
          <Modal
            title={<FormattedMessage id="文件复制错误" />}
            visible={
              !this.props.editingProgramName &&
              !!this.props.renamingProgramFailure
            }
            onCancel={this.props.dspResetRenameProgramStatus}
            onOk={this.props.dspResetRenameProgramStatus}
            okText={<FormattedMessage id="确认" />}
            cancelButtonProps={{ style: { display: "none" } }}
          >
            <div className={"error"}>
              {!!this.props.renamingProgramFailure &&
                this.props.renamingProgramFailure}
            </div>
          </Modal>
        )}
      </Fragment>
    );
  }

  renderContent = ({ getPrefixCls }) => {
    console.log("render Blockly");

    const {
      prefixCls: customizePrefixCls,
      cell,
      size: { width, height },
    } = this.props;
    const prefixCls = getPrefixCls("blockly", customizePrefixCls);
    const options = this.getOptions();
    const sidebarWidth = options.sidebarWidth || 50;
    const selectedProgram =
      this.props.programs[this.props.cell.options.selectedProgramName];

    return (
      <IntlProvider>
        <div className={`${prefixCls}`}>
          {(!options.hideSidebar || !selectedProgram) && (
            <ResizableBox
              width={sidebarWidth}
              height={height}
              minConstraints={[50, height]}
              maxConstraints={[width / 3, height]}
              className={`${prefixCls}-left-col`}
              axis={"x"}
              handle={<div className={`${prefixCls}-hdivider`} />}
              onResize={(e, { size: { width, height } }) => {
                this.updateOptions({ ...options, sidebarWidth: width });
              }}
            >
              {this.renderPrograms(prefixCls)}
            </ResizableBox>
          )}
          <div
            className={`${prefixCls}-right-col`}
            style={{
              width: !options.hideSidebar ? width - sidebarWidth : width,
            }}
          >
            {!!selectedProgram && (
              <Editors
                program={selectedProgram}
                fileName={
                  this.props.stateCode === 400
                    ? this.props.currentFile.slice(
                        0,
                        this.props.currentFile.length - 5
                      )
                    : this.props.cell.options.selectedFileName
                }
                cell={cell}
                prefixCls={prefixCls}
                updateOptions={(options) => this.updateOptions(options)}
                options={this.getOptions()}
              />
            )}
            {!selectedProgram && (
              <div className={`${prefixCls}-no-program`}>
                <span>
                  <FormattedMessage id="没有程序，请选中或新建." />
                </span>
              </div>
            )}
          </div>
        </div>
      </IntlProvider>
    );
  };
}

function mapStateToProps(state, props) {
  return {
    programs: state.robot.programs,
    stateCode: getStateCode(state),
    currentFile: getProgramFile(state),
    showCreateProgramModal: state.robot.showCreateProgramModal,
    creatingProgram: state.robot.creatingProgram,
    creatingProgramFailure: state.robot.creatingProgramFailure,
    creatingProgramSuccess: state.robot.creatingProgramSuccess,
    showDeleteProgramModal: state.robot.showDeleteProgramModal,
    deletingProgram: state.robot.deletingProgram,
    deletingProgramFailure: state.robot.deletingProgramFailure,
    deletingProgramSuccess: state.robot.deletingProgramSuccess,
    editingProgramName: state.robot.editingProgramName,
    renamingProgram: state.robot.renamingProgram,
    renamingProgramFailure: state.robot.renamingProgramFailure,
    renamingProgramSuccess: state.robot.renamingProgramSuccess,
  };
}

function mapDispatchToProps(dispatch) {
  return {
    dspLoadPrograms: (...args) => dispatch(loadPrograms(...args)),
    dspToggleCreateProgramModal: (...args) =>
      dispatch(toggleCreateProgramModal(...args)),
    dspCreateProgram: (...args) => dispatch(createProgram(...args)),
    dspToggleDeleteProgramModal: (...args) =>
      dispatch(toggleDeleteProgramModal(...args)),
    dspDeleteProgram: (...args) => dispatch(deleteProgram(...args)),
    dspToggleEditProgramName: (...args) =>
      dispatch(toggleEditProgramName(...args)),
    dspRenameProgram: (...args) => dispatch(renameProgram(...args)),
    dspResetRenameProgramStatus: (...args) =>
      dispatch(resetRenameProgramStatus(...args)),
  };
}

const ConnectedBlockly = connect(
  mapStateToProps,
  mapDispatchToProps
)(withSize({ monitorHeight: true, refreshRate: 30 })(Blockly));

ConnectedBlockly.NAME = "可视化编程";
ConnectedBlockly.WIDTH = 24;
ConnectedBlockly.HEIGHT = 12;
ConnectedBlockly.ICON = (props) => (
  <FileTextOutlined className={"icon"} {...props} />
);

export default ConnectedBlockly;
