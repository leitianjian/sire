import React from "react";
import PropTypes from "prop-types";
import { connect } from "react-redux";
import CellBase from "../CellBase";
import { DeleteFilled } from "@ant-design/icons";
import Button from "antd/lib/button";
import { sendCmd } from "../../../state/actions";
import Form from "antd/lib/form";
import Input from "antd/lib/input";
import Modal from "antd/lib/modal";
const layout = {
  labelCol: {
    span: 8,
  },
  wrapperCol: {
    span: 16,
  },
};

class ExperCell extends CellBase {
  formRef = React.createRef();

  constructor(props) {
    super(props);
    this.options = this.getOptions();

    if (this.options.instructObjectJSON === undefined) {
      this.instructObjectJSON = [];
    } else {
      this.instructObjectJSON = JSON.parse(this.options.instructObjectJSON);
    }
    this.state = {
      show: false,
      inputValue: "",
      inputDefault: "",
      instructObject: this.instructObjectJSON || [],
      compileNum: null,
      sendCommand: "",
      buttonValue: "发送",
      valueInputNum: 1,
      returnCode: 0,
    };
  }

  showState = () => {
    this.setState({
      show: true,
    });
  };

  sure = (e) => {
    e.preventDefault();
    const { instructObject } = this.state;
    this.formRef.current.validateFields().then((values) => {
      const rawCmd = values.instruct;
      const list = rawCmd.trim().split(/\$\{val[0-9]*\}/);
      const parametersValue = [];
      if (list.length !== 1) {
        list.pop();
        for (let i = 0; i < list.length; i++) {
          parametersValue.push(null);
        }
      }
      instructObject.push({
        name: values.name,
        instruct: values.instruct,
        parametersValue,
      });
      this.setState({ instructObject });

      this.setState({
        show: false,
      });
      const instructObjectJSON = JSON.stringify(instructObject);
      const options = this.getOptions();
      this.updateOptions({
        ...options,
        instructObjectJSON,
      });
    });
  };

  cancel = () => {
    this.setState({
      show: false,
    });
  };

  textChange = (e) => {
    this.setState({
      inputValue: e.target.value,
    });
  };

  send = (e, index) => {
    const { instructObject } = this.state;

    // 获取到cmd对象
    const rawCmd = instructObject[index].instruct;
    let cmdString = "";
    const splitList = [];

    if (rawCmd !== "" || rawCmd !== undefined) {
      const list = rawCmd.trim().split(/\$\{val[0-9]*\}/);
      if (list.length === 1) {
        cmdString += list[0];
      } else {
        list.pop();
        for (let i = 0; i < list.length; i++) {
          splitList.push(list[i]);
        }
        splitList.map((cmd, i) => {
          if (instructObject[index].parametersValue[i]) {
            cmdString += cmd + instructObject[index].parametersValue[i];
          }
        });
      }
    }
    this.setState({
      sendCommand: cmdString,
    });

    this.props.sendCmd(cmdString, (msg) => {
      this.setState({ returnCode: msg.jsData });
    });
  };

  textValue = (e) => {
    this.setState({
      buttonValue: e.target.value,
    });
  };

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;

    const { instructObject, sendCommand } = this.state;
    const that = this;
    const prefixCls = getPrefixCls("exper-cell", customizePrefixCls);
    return (
      <div className={`${prefixCls}`}>
        <div className={`${prefixCls}-content`}>
          {instructObject.map((value, index) => (
            <div className={`${prefixCls}-column-div`}>
              <Button
                id={index}
                type="primary"
                onClick={(e) => {
                  this.send(e, index);
                }}
              >
                {instructObject[index].name}
              </Button>
              <div
                className={`${prefixCls}-content-icon`}
                onClick={(e) => {
                  instructObject.splice(index, 1);
                  this.setState({ instructObject });
                  const instructObjectJSON = JSON.stringify(instructObject);
                  const options = this.getOptions();
                  this.updateOptions({
                    ...options,
                    instructObjectJSON,
                  });
                }}
              >
                <DeleteFilled className={`${prefixCls}-icon`} />
              </div>
              {instructObject[index].parametersValue.map((value, i) => (
                <Input
                  className={`${prefixCls}-content-input`}
                  placeholder={`参数${i}`}
                  defaultValue={instructObject[index].parametersValue[i]}
                  onChange={(e) => {
                    instructObject[index].parametersValue[i] = e.target.value;
                    this.setState({ instructObject });
                    const instructObjectJSON = JSON.stringify(instructObject);
                    const options = this.getOptions();
                    this.updateOptions({
                      ...options,
                      instructObjectJSON,
                    });
                  }}
                />
              ))}
            </div>
          ))}
        </div>
        <div className={`${prefixCls}-footer`}>
          {
            <div style={{ float: "right", marginRight: "20px" }}>
              <span>
                <Button onClick={this.showState}>增加</Button>
              </span>
            </div>
          }
          {!!sendCommand && (
            <span
              className={`${prefixCls}-footer-instruct-${
                this.state.returnCode && this.state.returnCode.return_code === 0
                  ? "success"
                  : this.state.returnCode &&
                    this.state.returnCode.return_code !== 0
                  ? "error"
                  : "sending"
              }`}
            >
              发送的指令：{that.state.sendCommand}
            </span>
          )}
        </div>
        <Modal
          title={"创建指令"}
          visible={!!this.state.show}
          onCancel={this.cancel}
          onOk={this.sure}
          okText={"确认"}
          cancelButtonProps={{ style: { display: "none" } }}
        >
          <Form {...layout} ref={this.formRef}>
            <Form.Item
              label="请输入指令名称："
              name={"name"}
              rules={[
                { type: "string", required: true, message: "请输入指令名称" },
              ]}
            >
              <Input
                size={"middle"}
                placeholder="请输入指令名称"
                value={this.state.buttonValue}
                onChange={this.textValue.bind(this)}
              />
            </Form.Item>
            <Form.Item
              label="请输入指令："
              name={"instruct"}
              rules={[
                { type: "string", required: true, message: "请输入指令" },
              ]}
            >
              <Input
                size={"middle"}
                placeholder="指令格式：j1 --direction=${val0}"
                value={this.state.inputValue}
                onChange={this.textChange.bind(this)}
              />
            </Form.Item>
          </Form>
          <div className={"error"}></div>
        </Modal>
      </div>
    );
  };
}

ExperCell.propTypes = {
  get: PropTypes.object,
};

const mapStateToProps = (state, props) => {
  return {
    // returnCode: getResultByChannel(state,`${props.cell.i}-experCmd`)
  };
};
// 关联redux和react
const ConnectedExperCell = connect(mapStateToProps, {
  sendCmd,
})(ExperCell);

ConnectedExperCell.NAME = "指令调试组件";
ConnectedExperCell.WIDTH = 24;
ConnectedExperCell.HEIGHT = 12;

export default ConnectedExperCell;
