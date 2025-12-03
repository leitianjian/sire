import React, { Component } from "react";
import PropTypes from "prop-types";
import Form from "antd/lib/form";
import Modal from "antd/lib/modal";
import Input from "antd/lib/input";
import { createSelector } from "reselect";
import { getlocalestate } from "../../../state/selectors";
import { connect } from "react-redux";

// 进行中英文切换
import zhCN from "../locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "../locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

const FormItem = Form.Item;

class CreateProgramModal extends Component {
  formRef = React.createRef();

  handleSubmit = (e) => {
    e.preventDefault();
    const { createProgram } = this.props;
    this.formRef.current.validateFields().then((values) => {
      createProgram(values);
    });
  };

  render() {
    const { toggleCreateProgramModal, prefixCls } = this.props;
    const { localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <Modal
          title={<FormattedMessage id="添加程序" />}
          visible={!!this.props.showCreateProgramModal}
          okText={<FormattedMessage id="创建" />}
          cancelText={<FormattedMessage id="取消" />}
          onOk={this.handleSubmit}
          onCancel={toggleCreateProgramModal}
          wrapClassName={`${prefixCls}-create-program-modal`}
        >
          <Form ref={this.formRef}>
            <FormItem
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
              <Input size={"small"} disabled={!!this.props.creatingProgram} />
            </FormItem>
          </Form>
          <div className={`${prefixCls}-status`}>
            {!!this.props.creatingProgram && (
              <div className={"info"}>{<FormattedMessage id="创建中" />}</div>
            )}
            {!!this.props.creatingProgramFailure && (
              <div className={"error"}>
                {<FormattedMessage id="创建错误：" />}
                {this.props.creatingProgramFailure}
              </div>
            )}
            {!!this.props.creatingProgramSuccess && (
              <div className={"success"}>
                {<FormattedMessage id="创建成功" />}
              </div>
            )}
          </div>
        </Modal>
      </IntlProvider>
    );
  }
}

const getBlocklyState = createSelector(getlocalestate, (localestate) => {
  return {
    localestate,
  };
});

function mapStateToProps(state, props) {
  return getBlocklyState(state, props);
}

CreateProgramModal.propTypes = {
  createProgram: PropTypes.func.isRequired,
  toggleCreateProgramModal: PropTypes.func.isRequired,
  prefixCls: PropTypes.string.isRequired,
};

const ConnectedCreateProgramModal =
  connect(mapStateToProps)(CreateProgramModal);

export default ConnectedCreateProgramModal;
