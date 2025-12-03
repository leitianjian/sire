import React, { Fragment } from "react";
import ConfigBase from "./ConfigBase";
import Form from "antd/lib/form";
import Modal from "antd/lib/modal";
import Radio from "antd/lib/radio";
import Select from "antd/lib/select";
import Button from "antd/lib/button";
import Icon from "antd/lib/icon";
import Row from "antd/lib/row";
import Col from "antd/lib/col";
import Input from "antd/lib/input";
import InputNumber from "antd/lib/input-number";
import { connect } from "react-redux";
import { createSelector } from "reselect";
import isEqual from "lodash/isEqual";
import {
  getMemoizeDataSources,
  getMemoizeDataIndexesBySource,
  getlocalestate,
} from "../../../state/selectors";
import {
  sendCmd,
  // deleteResultByChannel
} from "../../../state/actions";

// 进行中英文切换
import zhCN from "../locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "../locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

const { Search } = Input;

const RadioGroup = Radio.Group;
const FormItem = Form.Item;
const Option = Select.Option;

class Config extends ConfigBase {
  formRef = React.createRef();
  state = {
    sources: [],
  };

  componentDidUpdate(prevProps, prevState, snapshot) {
    const { cell: oldCell, dataSources: oldDataSources } = prevProps;
    const { cell: newCell, dataSources } = this.props;
    if (!isEqual(oldCell, newCell) && newCell && newCell.options) {
      if (newCell.options && newCell.options.sources) {
        try {
          this.setState({ sources: newCell.options.sources });
        } catch (e) {}
      }
    }
    if (!isEqual(oldDataSources, dataSources)) {
      this.setState({
        sources: this.state.sources.filter((s) => {
          if (s.source && dataSources.indexOf(s.source) === -1) {
            return false;
          }
          return true;
        }),
      });
    }
  }

  handleSubmit = (e) => {
    e.preventDefault();
    this.formRef.current.validateFields((err, fieldsValue) => {
      if (err) {
        return;
      }
      const options = this.getOptions();
      this.updateOptions({
        ...options,
        ...fieldsValue,
        sources: fieldsValue.sources,
        showConfig: false,
      });
    });
  };

  remove = (k) => {
    this.setState({
      sources: this.state.sources.filter((s, i) => i !== k),
    });
  };

  add = () => {
    this.setState({
      sources: [...this.state.sources, {}],
    });
  };

  renderDataSources(options) {
    const { getFieldsValue } = this.formRef.current;
    const { dataSources, dataIndexes } = this.props;
    const selectedSources = options.sources || [];
    selectedSources.forEach((source, index) => {
      return { name: `sources[${index}].source`, initialValue: source.source };
    });
    const { sources } = this.state;
    if (!dataSources || !dataIndexes) {
      return (
        <div>
          <FormattedMessage id="数据等待中..." />
        </div>
      );
    }
    if (Object.keys(dataSources).length === 0) {
      return (
        <div style={{ textAlign: "center" }} className={"error"}>
          <FormattedMessage id="无可添加数据项，请输入并运行指令，并确认返回结果中包含数值。" />
        </div>
      );
    }
    let { sources: formSources } = getFieldsValue();
    if (!formSources) {
      formSources = [];
    }
    const sourceItems = sources.map((source, index) => {
      return (
        <Row key={index}>
          <Col span={10}>
            <FormItem
              label={<FormattedMessage id="数据项" />}
              labelCol={{ span: 8 }}
              wrapperCol={{ span: 16 }}
              name={`sources[${index}].source`}
              initialValue={source.source}
              rules={[
                {
                  type: "string",
                  required: true,
                  message: <FormattedMessage id="请选择数据项" />,
                },
              ]}
            >
              <Select
                style={{ width: "100%" }}
                size={"small"}
                placeholder={<FormattedMessage id="选择数据项" />}
              >
                {dataSources.map((c) => (
                  <Option key={c} value={c}>
                    {c}
                  </Option>
                ))}
              </Select>
            </FormItem>
          </Col>
          <Col span={10}>
            {!!formSources[index] && !!dataIndexes[formSources[index].source] && (
              <FormItem
                label={"index"}
                labelCol={{ span: 8 }}
                wrapperCol={{ span: 16 }}
                initialValue={source.index}
                rules={[
                  {
                    type: "number",
                    required: true,
                    message: <FormattedMessage id="请选择index" />,
                  },
                ]}
              >
                <Select
                  style={{ width: "100%" }}
                  size={"small"}
                  placeholder={<FormattedMessage id="选择index" />}
                >
                  {dataIndexes[formSources[index].source].map((c) => (
                    <Option key={c.toString()} value={c}>
                      {c}
                    </Option>
                  ))}
                </Select>
              </FormItem>
            )}
          </Col>
          <Col span={4}>
            <Form.Item style={{ paddingLeft: 20 }}>
              <Button onClick={this.remove.bind(this, index)} type={"danger"}>
                <Icon type="minus" />
                <FormattedMessage id="移除" />
              </Button>
            </Form.Item>
          </Col>
        </Row>
      );
    });
    return (
      <Fragment>
        {sourceItems}
        <Form.Item wrapperCol={{ span: 24 }} style={{ textAlign: "center" }}>
          <Button type="dashed" onClick={this.add}>
            <Icon type="plus" />
            <FormattedMessage id="添加数据项" />
          </Button>
        </Form.Item>
      </Fragment>
    );
  }

  renderContent = ({ getPrefixCls }) => {
    const {
      prefixCls: customizePrefixCls,
      cell,
      sendCmd,
      localestate,
    } = this.props;
    const options = this.getOptions();
    const prefixCls = getPrefixCls("realtime-chart-config", customizePrefixCls);
    const interval = options.interval || 1000;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <Modal
          title={<FormattedMessage id="添加数据" />}
          visible={!!options.showConfig}
          maskClosable={false}
          okText={<FormattedMessage id="保存" />}
          cancelText={<FormattedMessage id="取消" />}
          width={"80%"}
          onOk={this.handleSubmit}
          onCancel={this.hideConfig}
          wrapClassName={`${prefixCls}`}
        >
          <Form layout={"horizontal"}>
            <FormItem
              label={<FormattedMessage id="数据来源" />}
              name={"channel"}
              initialValue={options.channel === "command" ? "command" : "get"}
              rules={[
                {
                  type: "string",
                  required: true,
                  message: <FormattedMessage id="请选择数据来源" />,
                },
              ]}
            >
              <RadioGroup
                onChange={(e) => {
                  this.updateOptions({ ...options, channel: e.target.value });
                }}
              >
                <Radio value={"get"}>
                  <FormattedMessage id="默认GET" />
                </Radio>
                <Radio value={"command"}>
                  <FormattedMessage id="自定义命令" />
                </Radio>
              </RadioGroup>
            </FormItem>
            {options.channel === "command" && (
              <Row>
                <Col span={20}>
                  <FormItem
                    label={<FormattedMessage id="运行指令" />}
                    labelCol={{ span: 4 }}
                    wrapperCol={{ span: 20 }}
                    name={"command"}
                    initialValue={options.command}
                    rules={[
                      {
                        type: "string",
                        required: true,
                        message: <FormattedMessage id="请输入指令" />,
                      },
                    ]}
                  >
                    <Search
                      onSearch={(value) => {
                        sendCmd(value, cell.i);
                      }}
                      enterButton={<FormattedMessage id="运行" />}
                    />
                  </FormItem>
                </Col>
                <Col span={4}>
                  <FormItem
                    label={<FormattedMessage id="运行间隔" />}
                    labelCol={{ span: 12 }}
                    wrapperCol={{ span: 12 }}
                    name={"interval"}
                    initialValue={interval}
                    rules={[
                      {
                        required: true,
                        message: <FormattedMessage id="请输入指令" />,
                      },
                    ]}
                  >
                    <InputNumber
                      min={30}
                      step={1}
                      formatter={(value) => `${value}ms`}
                      parser={(value) => value.replace("ms", "")}
                    />
                  </FormItem>
                </Col>
              </Row>
            )}
            {this.renderDataSources(options)}
          </Form>
        </Modal>
      </IntlProvider>
    );
  };
}

const getState = createSelector(
  getMemoizeDataSources,
  getMemoizeDataIndexesBySource,
  getlocalestate,
  (dataSources, dataIndexes, localestate) => {
    return {
      dataSources,
      dataIndexes,
      localestate,
    };
  }
);

function mapStateToProps(state, props) {
  return getState(state, props);
}

// const transform = (obj) => {
//   return Object.keys(obj).reduce((acc, cv) => {
//     return {
//       ...acc,
//       [cv]:
//         typeof obj[cv] === "object" && !("value" in obj[cv])
//           ? isArray(obj[cv])
//             ? obj[cv].map((item) => transform(item))
//             : transform(obj[cv])
//           : Form.createFormField({
//               ...obj[cv],
//               value: obj[cv] && obj[cv].value,
//             }),
//     };
//   }, {});
// };

export default connect(mapStateToProps, {
  sendCmd,
  // deleteResultByChannel,
})(Config);
