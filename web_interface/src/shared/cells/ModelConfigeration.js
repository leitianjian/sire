import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import PropTypes from "prop-types";
import Row from "antd/lib/row";
import Input from "antd/lib/input";
import Button from "antd/lib/button";
import Select from "antd/lib/select";
import Upload from "antd/lib/upload";
import {
  FileMarkdownOutlined,
  CloseCircleOutlined,
  UploadOutlined,
} from "@ant-design/icons";
import Modal from "antd/lib/modal";
import Form from "antd/lib/form";

import { connect } from "react-redux";
import { createSelector } from "reselect";
import CellBase from "./CellBase";
import {
  getCreateRobotModelModalVisible,
  getCreatingRobotModel,
  getCreatingRobotModelError,
  getCreatingRobotModelResult,
  getCurrentRobotBrand,
  getCurrentRobotModel,
  getRobotModels,
  getlocalestate,
  getPath,
} from "../../state/selectors";
import {
  // deleteResultByChannel,
  loadRobotModels,
  robotModelPartUploadSuccess,
  deleteRobotModelPart,
  deleteRobotModel,
  createRobotModel,
  showCreateRobotModelModal,
  hideCreateRobotModelModal,
  selectCurrentRobotBrand,
  selectCurrentRobotModel,
} from "../../state/actions";

// 进行中英文切换
import zhCN from "./locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "./locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

const Option = Select.Option;

const formItemLayout = {
  labelCol: { span: 4 },
  wrapperCol: { span: 8 },
};
const FormItem = Form.Item;

class CreateRobotForm extends Component {
  formRef = React.createRef();

  handleSubmit = (e) => {
    e.preventDefault();
    const { createRobotModel } = this.props;
    this.formRef.current.validateFields((err, values) => {
      if (!err) {
        createRobotModel(values);
      }
    });
  };

  render() {
    const {
      visible,
      creatingRobotModel,
      creatingRobotModelError,
      hideCreateRobotModelModal,
    } = this.props;
    return (
      <Modal
        visible={visible}
        title={<FormattedMessage id="创建机器人" />}
        onOk={this.handleSubmit}
        okText={<FormattedMessage id="创建" />}
        cancelText={<FormattedMessage id="取消" />}
        onCancel={() => {
          hideCreateRobotModelModal();
        }}
        centered={true}
      >
        {
          <Form onSubmit={this.handleSubmit} className="login-form">
            <FormItem
              {...formItemLayout}
              label={<FormattedMessage id="品牌" />}
              name={"brand"}
              rules={[
                {
                  required: true,
                  message: <FormattedMessage id="请输入机器人品牌" />,
                },
              ]}
            >
              <Input placeholder="Brand" />
            </FormItem>
            <FormItem
              {...formItemLayout}
              label={<FormattedMessage id="型号" />}
              name={"model"}
              rules={[
                {
                  required: true,
                  message: <FormattedMessage id="请输入机器人型号" />,
                },
              ]}
            >
              <Input placeholder="Model" />
            </FormItem>
          </Form>
        }
        {!!creatingRobotModel && (
          <span className={"info"}>
            <FormattedMessage id="创建中..." />
          </span>
        )}
        {!!creatingRobotModelError && (
          <span className={"error"}>{creatingRobotModelError}</span>
        )}
      </Modal>
    );
  }
}

const CreateRobotModel = CreateRobotForm;

class ModelConfiguration extends CellBase {
  name = "三维模型配置";
  state = {
    showCalibResult: false,
    showSaveResult: false,
    brand: null,
    model: null,
    showCreateModal: false,
  };

  componentDidMount() {
    this.props.loadRobotModels();
  }

  renderContent = ({ getPrefixCls }) => {
    const self = this;
    let {
      prefixCls: customizePrefixCls,
      cell,
      // deleteResultByChannel,
      robotModels,
      robotModelPartUploadSuccess,
      deleteRobotModelPart,
      deleteRobotModel,
      creatingRobotModel,
      creatingRobotModelError,
      showCreateRobotModelModal,
      modalVisible,
      createRobotModel,
      selectCurrentRobotBrand,
      selectCurrentRobotModel,
      hideCreateRobotModelModal,
      model,
      brand,
      savePath,
    } = self.props;
    if (savePath) {
      savePath = savePath.replace(/\\/g, "/");
    }
    console.log("savePath", savePath);
    const prefixCls = getPrefixCls("model-configuration", customizePrefixCls);
    if (!robotModels) {
      return (
        <div className={`${prefixCls} loading`}>
          <FormattedMessage id="参数载入中..." />
        </div>
      );
    }
    if (!brand || !robotModels[brand]) {
      brand = Object.keys(robotModels)[0];
    }
    if (brand) {
      if (!model || !robotModels[brand][model]) {
        model = Object.keys(robotModels[brand])[0];
      }
    }
    // console.log("path",savePath)
    // let missingParts = []
    // if(brand && model){
    //   missingParts = Object.keys(robotModels[brand][model].parts).filter(key=>!robotModels[brand][model].parts[key])
    // }
    return (
      <div className={prefixCls}>
        {!!robotModels && model && brand && (
          <Row className={`${prefixCls}-brand`} style={{ marginBottom: 20 }}>
            <span>
              <FormattedMessage id="机器人品牌" />
            </span>
            <Select
              size={"small"}
              style={{ width: 200 }}
              placeholder={<FormattedMessage id="选择机器人品牌" />}
              onChange={(brand) => {
                this.setState({ saving: false });
                // deleteResultByChannel(cell.i)
                selectCurrentRobotBrand(brand);
              }}
              value={brand}
            >
              {Object.keys(robotModels).map((brand) => (
                <Option value={brand} key={brand}>
                  {brand}
                </Option>
              ))}
            </Select>
          </Row>
        )}
        {!!robotModels && model && brand && model && !!robotModels[brand] && (
          <Row className={`${prefixCls}-model`} style={{ marginBottom: 20 }}>
            <span>
              <FormattedMessage id="机器人型号" />
            </span>
            <Select
              size={"small"}
              style={{ width: 200 }}
              placeholder={<FormattedMessage id="选择机器人型号" />}
              onChange={(model) => {
                this.setState({ saving: false });
                // deleteResultByChannel(cell.i)
                selectCurrentRobotModel(model);
              }}
              value={model}
            >
              {Object.keys(robotModels[brand]).map((model) => (
                <Option value={model} key={model}>
                  {model}
                </Option>
              ))}
            </Select>
            <Button
              style={{ marginLeft: 10, marginBottom: 20 }}
              size={"small"}
              onClick={() => {
                this.setState({ saving: false });
                // deleteResultByChannel(cell.i)
                deleteRobotModel(brand, model);
              }}
            >
              <FormattedMessage id="删除" />
            </Button>
          </Row>
        )}
        <Row className={`${prefixCls}-model`}>
          <Button
            size={"small"}
            onClick={() => {
              this.setState({ saving: false });
              // deleteResultByChannel(cell.i)
              showCreateRobotModelModal();
            }}
          >
            <FormattedMessage id="添加机器人" />
          </Button>
          <CreateRobotModel
            visible={modalVisible}
            hideCreateRobotModelModal={hideCreateRobotModelModal}
            centered={true}
            createRobotModel={createRobotModel}
            creatingRobotModel={creatingRobotModel}
            creatingRobotModelError={creatingRobotModelError}
          ></CreateRobotModel>
        </Row>
        <Row className={`${prefixCls}-parts`}>
          {!!robotModels &&
            !!robotModels[brand] &&
            !!robotModels[brand][model] &&
            !!robotModels[brand][model].parts &&
            Object.keys(robotModels[brand][model].parts).map((key) => {
              const fileList = [];
              if (robotModels[brand][model].parts[key]) {
                fileList.push({
                  uid: "-1",
                  name: robotModels[brand][model].parts[key],
                  status: "done",
                  url: "",
                });
              }
              return (
                <Row
                  className={`${prefixCls}-part ${
                    this.state.saving &&
                    !robotModels[brand][model].parts[key] &&
                    "error"
                  }`}
                  key={key}
                >
                  {<span>PART {key}:</span>}
                  {!!robotModels[brand][model].parts[key] && (
                    <span className={`${prefixCls}-part-file`}>
                      <FileMarkdownOutlined />{" "}
                      {robotModels[brand][model].parts[key]}
                      <CloseCircleOutlined
                        className={`${prefixCls}-delete`}
                        onClick={() => {
                          deleteRobotModelPart(brand, model, key);
                        }}
                      />
                    </span>
                  )}
                  {!robotModels[brand][model].parts[key] && (
                    <Upload
                      className={`${prefixCls}-part-upload`}
                      multiple={false}
                      action={`/api/robots/${brand}/${model}/parts/${key}`}
                      showUploadList={false}
                      onChange={({ file, fileList, event }) => {
                        if (file.status === "done") {
                          robotModelPartUploadSuccess(file.response);
                        }
                      }}
                    >
                      <Button>
                        <UploadOutlined />
                        <FormattedMessage id="上传" />
                      </Button>
                    </Upload>
                  )}
                </Row>
              );
            })}
        </Row>
        <Row className={`${prefixCls}-save`}>
          <Button
            size={"large"}
            type={"primary"}
            onClick={() => {
              this.setState({ saving: true });
              // deleteResultByChannel(cell.i)
              // if(missingParts.length === 0){
              const parts = robotModels[brand][model].parts;

              const path = robotModels[brand][model].path;
              this.sendCmd("cs_stop");
              const cmd =
                "setppath --file_path=" +
                Object.keys(parts)
                  .map((key) => {
                    // console.log("parts",parts[key])
                    // if(!!parts[key]){
                    return `/${path}/${parts[key]}`;
                    // }else{
                    // return ""
                    // }
                  })
                  .join(";");
              this.sendCmd(cmd);
              this.sendCmd("cs_stop", `${cell.i}-cs_stop`);
              // this.sendCmd(`savexml --path=` + `${savePath}/robot`)
              this.sendCmd("savexml", `${cell.i}-savexml`);
              // }
            }}
          >
            <FormattedMessage id="保存" />
          </Button>
          {/* {this.state.saving && missingParts.length > 0 && <p className={'error'}>
        <FormattedMessage id = "请上传机器人模型"/>
        {missingParts.join(',')}
        </p> } */}
        </Row>
      </div>
    );
  };

  render() {
    const { localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <ConfigConsumer>{this.renderContent}</ConfigConsumer>
      </IntlProvider>
    );
  }
}

ModelConfiguration.propTypes = {
  result: PropTypes.object,
};

const getModelConfigerationState = createSelector(
  getRobotModels,
  getCreatingRobotModel,
  getCreatingRobotModelError,
  getCreatingRobotModelResult,
  getCreateRobotModelModalVisible,
  getCurrentRobotBrand,
  getCurrentRobotModel,
  getlocalestate,
  getPath,
  (
    robotModels,
    creatingRobotModel,
    creatingRobotModelError,
    creatingRobotModelResult,
    modalVisible,
    brand,
    model,
    localestate,
    savePath
  ) => {
    return {
      robotModels,
      creatingRobotModel,
      creatingRobotModelError,
      creatingRobotModelResult,
      modalVisible,
      brand,
      model,
      localestate,
      savePath,
    };
  }
);

function mapStateToProps(state, props) {
  return getModelConfigerationState(state, props);
}

const ConnectedModelConfiguration = connect(mapStateToProps, {
  // deleteResultByChannel,
  loadRobotModels,
  robotModelPartUploadSuccess,
  deleteRobotModelPart,
  deleteRobotModel,
  createRobotModel,
  showCreateRobotModelModal,
  hideCreateRobotModelModal,
  selectCurrentRobotBrand,
  selectCurrentRobotModel,
})(ModelConfiguration);

ConnectedModelConfiguration.NAME = "三维模型配置";

export default ConnectedModelConfiguration;
