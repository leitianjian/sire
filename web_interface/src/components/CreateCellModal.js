import React, { PureComponent } from "react";
import Form from "antd/lib/form";
import Modal from "antd/lib/modal";
import Radio from "antd/lib/radio";
import { BlockOutlined } from "@ant-design/icons";
import Input from "antd/lib/input";
import InputNumber from "antd/lib/input-number";
import Cells from "../shared/cells/index";
import { connect } from "react-redux";
import { IntlProvider, FormattedMessage } from "react-intl";

import { actCreateCell, actToggleCreateCellModal } from "../state/actions";

const FormItem = Form.Item;
const RadioGroup = Radio.Group;
const RadioButton = Radio.Button;

class CreateCellModal extends PureComponent {
  formRef = React.createRef();

  handleSubmit = (e) => {
    e.preventDefault();
    this.formRef.current.validateFields().then((fieldsValue) => {
      this.props.dspCreateCell(this.props.pathname.substring(12), fieldsValue);
    });
  };

  render() {
    const { prefixCls } = this.props;
    // console.log("test creatingCell", Cells);
    return (
      <IntlProvider>
        <Modal
          title={<FormattedMessage id="添加面板" />}
          open={this.props.prpShowCreateCellModal}
          width={"75%"}
          okText={<FormattedMessage id="创建" />}
          cancelText={<FormattedMessage id="取消" />}
          onOk={this.handleSubmit}
          onCancel={this.props.dspToggleCreateCellModal}
          wrapClassName={`${prefixCls}-create-cell-modal`}
        >
          <Form ref={this.formRef}>
            <FormItem
              label={<FormattedMessage id="类型" />}
              className={`${prefixCls}-type-selector`}
              name={"type"}
              rules={[
                {
                  type: "string",
                  required: true,
                  message: <FormattedMessage id="请选择面板类型!" />,
                },
              ]}
            >
              <RadioGroup
                className={`${prefixCls}-type-selector-radio-group`}
                disabled={!!this.props.prpCreatingCell}
                onChange={(e) => {
                  const s = e.target.value;
                  this.formRef.current.setFieldsValue({
                    name: `${Cells[s].NAME}`,
                  });
                  this.formRef.current.setFieldsValue({
                    w: Cells[s].WIDTH ? Cells[s].WIDTH : 24,
                  });
                  this.formRef.current.setFieldsValue({
                    h: Cells[s].HEIGHT ? Cells[s].HEIGHT : 12,
                  });
                }}
              >
                {Object.keys(Cells).map((key) => {
                  if (key !== "Unknown") {
                    const ICON = Cells[key].ICON
                      ? Cells[key].ICON
                      : (props) => (
                          <BlockOutlined className={"icon"} {...props} />
                        );
                    return (
                      <RadioButton value={key} key={key}>
                        <div>
                          <div>
                            {" "}
                            <ICON />{" "}
                          </div>
                          <div className={`${prefixCls}-type-name`}>
                            <FormattedMessage
                              id={Cells[key].NAME ? Cells[key].NAME : key}
                            />
                          </div>
                        </div>
                      </RadioButton>
                    );
                  }
                })}
              </RadioGroup>
            </FormItem>
            <FormItem
              label={<FormattedMessage id="名称" />}
              className={`${prefixCls}-input-col`}
              name={"name"}
              initialValue={"组件"}
              rules={[
                {
                  type: "string",
                  required: true,
                  message: <FormattedMessage id="请输入面板名称" />,
                },
              ]}
            >
              <Input size={"small"} disabled={!!this.props.prpCreatingCell} />
            </FormItem>
            <FormItem
              label={<FormattedMessage id="宽度" />}
              className={`${prefixCls}-input-col`}
              name={"w"}
              initialValue={24}
              rules={[
                {
                  type: "integer",
                  required: true,
                  message: <FormattedMessage id="请输入面板宽度" />,
                },
              ]}
            >
              <InputNumber
                max={48}
                min={3}
                step={1}
                size={"small"}
                disabled={!!this.props.prpCreatingCell}
              />
            </FormItem>
            <FormItem
              label={<FormattedMessage id="高度" />}
              className={`${prefixCls}-input-col`}
              name={"h"}
              initialValue={12}
              rules={[
                {
                  type: "integer",
                  required: true,
                  message: <FormattedMessage id="请输入面板高度" />,
                },
              ]}
            >
              <InputNumber
                max={48}
                min={3}
                step={1}
                size={"small"}
                disabled={!!this.props.prpCreatingCell}
              />
            </FormItem>
          </Form>
          <div className={`${prefixCls}-status`}>
            {!!this.props.prpCreatingCell && (
              <div className={"info"}>
                {<FormattedMessage id="创建中..." />}
              </div>
            )}
            {!!this.props.prpCreatingCellFailure && (
              <div className={"error"}>
                {<FormattedMessage id="创建错误：" />}
                {this.props.creatingCellFailure}
              </div>
            )}
            {!!this.props.prpCreatingCellSuccess && (
              <div className={"success"}>
                {" "}
                {<FormattedMessage id="创建成功" />}
              </div>
            )}
          </div>
        </Modal>
      </IntlProvider>
    );
  }
}

function mapStateToProps(state) {
  return {
    pathname: state.router.pathname,
    prpShowCreateCellModal: state.config.showCreateCellModal,
    prpCreatingCell: state.config.creatingCell,
    prpCreatingCellFailure: state.config.creatingCellFailure,
    prpCreatingCellSuccess: state.config.creatingCellSuccess,
  };
}

const mapDispatchToProps = (dispatch) => {
  return {
    dspCreateCell: (...args) => dispatch(actCreateCell(...args)),
    dspToggleCreateCellModal: (...args) =>
      dispatch(actToggleCreateCellModal(...args)),
  };
};

export default connect(mapStateToProps, mapDispatchToProps)(CreateCellModal);
