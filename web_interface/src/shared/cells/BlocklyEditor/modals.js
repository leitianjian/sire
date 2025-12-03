import React, { Component } from "react";
import Form from "antd/lib/form";
import Modal from "antd/lib/modal";
import Input from "antd/lib/input";
import { connect } from "react-redux";

// 进行中英文切换
import zhCN from "../locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "../locale/enUS";

import { updateProgram } from "../../../state/actions";

const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

class CreateFileModalRaw extends Component {
  formRef = React.createRef();

  handleSubmit = (e) => {
    e.preventDefault();

    this.formRef.current.validateFields((err, fieldsValue) => {
      if (err) {
        return;
      }

      const newProgram = { ...this.props.program };
      newProgram.files.push({
        name: fieldsValue.name + ".pro",
        content: '<xml xmlns="https://developers.google.com/blockly/xml"/>',
      });
      newProgram.files.push({
        name: fieldsValue.name + ".dat",
        content: '<xml xmlns="https://developers.google.com/blockly/xml"/>',
      });
      newProgram.files.push({ name: fieldsValue.name + ".aris", content: "" });

      this.props.updateProgram(newProgram.name, newProgram);
      this.props.closeModal();
    });
  };

  render() {
    const { prefixCls } = this.props;
    const { localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <Modal
          wrapClassName={`${prefixCls}-create-program-modal`}
          title={<FormattedMessage id="添加程序" />}
          okText={<FormattedMessage id="创建" />}
          cancelText={<FormattedMessage id="取消" />}
          onOk={this.handleSubmit}
          visible={this.props.visible}
          onCancel={this.props.closeModal}
        >
          <Form ref={this.formRef}>
            <Form.Item
              label={<FormattedMessage id="名称" />}
              className={`${prefixCls}-input-col`}
              name={"name"}
              rules={[
                {
                  type: "string",
                  required: true,
                  message: `${(<FormattedMessage id="请输入程序名称" />)}`,
                },
              ]}
            >
              <Input size={"small"} />
            </Form.Item>
          </Form>
        </Modal>
      </IntlProvider>
    );
  }
}

function mapStateToProps(state) {
  return {
    updatingProgram: state.robot.updatingProgram,
    deletingProgramFailure: state.robot.deletingProgramFailure,
    deletingProgramSuccess: state.robot.deletingProgramSuccess,
  };
}

function mapDispatchToProps(dispatch) {
  return {
    updateProgram: (...args) => dispatch(updateProgram(...args)),
  };
}

export const CreateFileModal = connect(
  mapStateToProps,
  mapDispatchToProps
)(CreateFileModalRaw);
