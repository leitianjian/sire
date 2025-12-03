import React from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import PropTypes from "prop-types";
import { QuestionOutlined } from "@ant-design/icons";

import Row from "antd/lib/row";
import Col from "antd/lib/col";
import Button from "antd/lib/button";
import Select from "antd/lib/select";
import { connect } from "react-redux";
import Popover from "antd/lib/popover";
import Table from "antd/lib/table";
import InputNumber from "antd/lib/input-number";
import Form from "antd/lib/form";
import { createSelector } from "reselect";

import CellBase from "./CellBase";
import {
  getMemoizeEndPE,
  getToolIds,
  getlocalestate,
  getWobjIds,
  getMemoizeMotionPos,
  getCurrentToolId,
  getStateCode,
  getMemoizeCsStarted,
  getWebsocketConnected,
} from "../../state/selectors";
import { sendCmd } from "../../state/actions";

// 进行中英文切换
import zhCN from "./locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "./locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

const slaveFormItemLayout = {
  labelCol: {
    span: 10,
  },
  wrapperCol: {
    span: 14,
  },
};

const Option = Select.Option;
const FormItem = Form.Item;
const HINTS = {
  UserInput: "直接输入工具坐标系的位置和资态(欧拉角)",
  CalibWobj: "工件坐标系标定",
  CalibT4P:
    "以法兰坐标系为参考坐标系，标定出工具坐标系原点的位置偏移量，工具坐标系的姿态默认与法兰坐标系一致。",
  CalibT5P:
    "以法兰坐标系为参考坐标系，标定出工具坐标 系原点的位置偏移量以 及工具坐标系 Z 轴的方 向。",
  CalibT6P: "以法兰坐标系为参考坐标系，标定出工具坐标系的位姿。",
  CalibFC: "力传感器标定",
};
const DETAIL_HINTS = {
  CalibT4P: (
    <div
      style={{
        maxWidth: 300,
      }}
    >
      <p>
        1)
        <FormattedMessage id="选择法兰坐标系为当前工具坐标系，机器人底座坐标系为 当前工件坐标系; " />{" "}
      </p>
      <p>
        {" "}
        2)
        <FormattedMessage id="手动示教，以不同姿态使移动工具使工具中心点沿某一方 向与外部固定点接触，并保存此时的位姿，依次沿 4 个不同 方向获取示教点的位姿数据;" />
      </p>
      <p>
        3)
        <FormattedMessage id="确认标定结果。" />
      </p>
    </div>
  ),
  CalibT5P: (
    <div
      style={{
        maxWidth: 300,
      }}
    >
      <p>
        1)
        <FormattedMessage id="选择法兰坐标系为当前工具坐标系，机器人底座坐标系为 当前工件坐标系;" />
      </p>
      <p>
        2)
        <FormattedMessage id="手动示教，以不同姿态使移动工具使工具中心点沿某一方 向与外部固定点接触，并保存此时的位姿，依次沿 4 个不同 方向获取示教点的位姿数据;" />{" "}
      </p>
      <p>
        3)
        <FormattedMessage id="以第 4 点的工具位姿为基准，保持工具姿态不变，将工具 坐标系 Z 轴正方向上某一特征点与外部固定点接触，保存第 5 个示教点的位姿;" />{" "}
      </p>
      <p>
        4)
        <FormattedMessage id="确认标定结果。" />
      </p>
    </div>
  ),
  CalibT6P: (
    <div
      style={{
        maxWidth: 300,
      }}
    >
      <p>
        1)
        <FormattedMessage id="选择法兰坐标系为当前工具坐标系，机器人底座坐标系为 当前工件坐标系; " />
      </p>
      <p>
        2)
        <FormattedMessage id="手动示教，以不同姿态使移动工具使工具中心点沿某一方 向与外部固定点接触，并保存此时的位姿，依次沿 4 个不同 方向获取示教点的位姿数据;" />
      </p>
      <p>
        3)
        <FormattedMessage id="以第 4 点的工具位姿为基准，保持工具姿态不变，将工具 坐标系 Z 轴正方向上某一特征点与外部固定点接触，保存第 5 个示教点的位姿;" />
      </p>
    </div>
  ),
  UserInput: (
    <div
      style={{
        maxWidth: 300,
      }}
    >
      <p>
        <FormattedMessage id="用户输入标定 直接输入工具坐标系的位置和姿态（欧拉角）， 依次输入工具坐标系的位置和姿态，然后点击保存" />
      </p>
    </div>
  ),
};
const CALIB_METHODS = {
  UserInput: "用户输入",
  CalibWobj: "工件标定法",
  CalibT4P: "四点标定法",
  CalibT5P: "五点标定法",
  CalibT6P: "六点标定法",
  CalibFC: "力传感器标定法",
};
const POINTS_COUNT = {
  CalibWobj: 3,
  CalibT4P: 4,
  CalibT5P: 5,
  CalibT6P: 6,
  CalibFC: 3,
};

class ToolCalib extends CellBase {
  state = {
    calibMethod: window.localStorage.getItem("calibMethod") || "CalibT4P",
    toolId: Number(window.localStorage.getItem("toolId")) || 1,
    wobjId: Number(window.localStorage.getItem("wobjId")) || 1,
    toolName: window.localStorage.getItem("toolName") || "",
    pes: JSON.parse(window.localStorage.getItem("calibMethodData")) || [],
    result: {},
    showCalibResult: false,
    showSaveResult: false,
    userInput: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      a: 0.0,
      b: 0.0,
      c: 0.0,
    },

    calibData: {
      UserInput: [],
      CalibWobj: [],
      CalibT4P: [],
      CalibT5P: [],
      CalibT6P: [],
      CalibFC: [],
    },
    calibResult: "",
  };

  formRef = React.createRef();

  handleSelectToolId = (toolId) => {
    const calibMethod = this.state.calibMethod;
    this.setState({
      toolId,
      pes:
        JSON.parse(
          window.localStorage.getItem(`${calibMethod}-${toolId}-data`)
        ) || [],
      showCalibResult: false,
      toolName:
        window.localStorage.getItem(`${calibMethod}-${toolId}-name`) || null,
      showSaveResult: false,
    });
    // this.updateOptions({...options, "toolId":toolId})
    window.localStorage.setItem("toolId", toolId);
  };

  renderTable = (prefixCls) => {
    const render = (text, record) => {
      if (record.last) {
        return (
          <span
            className={`${prefixCls}-table-cell ${prefixCls}-table-cell-last`}
          >
            {text}
          </span>
        );
      } else {
        return <span className={`${prefixCls}-table-cell`}>{text}</span>;
      }
    };
    const columns = [
      {
        title: <FormattedMessage id="编号" />,
        dataIndex: "rowName",
        key: "rowName",
        align: "center",
      },
      {
        title: "X[mm]",
        dataIndex: "x",
        key: "xi",
        render,
        align: "center",
      },
      {
        title: "Y[mm]",
        dataIndex: "y",
        key: "yi",
        render,
        align: "center",
      },
      {
        title: "Z[mm]",
        dataIndex: "z",
        key: "zi",
        render,
        align: "center",
      },
      {
        title: "A",
        dataIndex: "a",
        key: "ai",
        render,
        align: "center",
      },
      {
        title: "B",
        dataIndex: "b",
        key: "bi",
        render,
        align: "center",
      },
      {
        title: "C",
        dataIndex: "c",
        key: "ci",
        render,
        align: "center",
      },
    ];
    const { endPE, localestate } = this.props;
    const data = [];
    this.state.calibData[this.state.calibMethod].forEach((pe, index) => {
      // map方法第一个参数为value,第二个为索引
      data.push({
        rowName: `${localestate === "chinese" ? "标定点" : "fixed point"}${
          index + 1
        }`,
        rowKey: index,
        key: index,
        xi: `x${index}`,
        yi: `y${index}`,
        zi: `z${index}`,
        ai: `a${index}`,
        bi: `b${index}`,
        ci: `c${index}`,
        x: (pe[0] * 1000).toFixed(3),
        y: (pe[1] * 1000).toFixed(3),
        z: (pe[2] * 1000).toFixed(3),
        a: ((pe[3] * 180) / Math.PI).toFixed(3),
        b: ((pe[4] * 180) / Math.PI).toFixed(3),
        c: ((pe[5] * 180) / Math.PI).toFixed(3),
      });
    });
    if (
      endPE &&
      this.state.calibData[this.state.calibMethod].length <
        POINTS_COUNT[this.state.calibMethod]
    ) {
      data.push({
        rowName: `${localestate === "chinese" ? "标定点" : "fixed point"}${
          data.length + 1
        }`,
        rowKey: "last",
        key: `last${data.length + 1}`,
        xi: "xlast",
        yi: "ylast",
        zi: "zlast",
        ai: "alast",
        bi: "blast",
        ci: "clast",
        x: (endPE[0] * 1000).toFixed(3),
        y: (endPE[1] * 1000).toFixed(3),
        z: (endPE[2] * 1000).toFixed(3),
        a: ((endPE[3] * 180) / Math.PI).toFixed(3),
        b: ((endPE[4] * 180) / Math.PI).toFixed(3),
        c: ((endPE[5] * 180) / Math.PI).toFixed(3),
        last: true,
      });
    }

    const ds =
      !this.props.isWsConnected ||
      !this.props.isCsStarted ||
      this.props.stateCode !== 200;

    const Footer = (
      <div className={`${prefixCls}-table-footer`}>
        {this.state.calibMethod === "CalibFC" && (
          <Form
            ref={this.formRef}
            style={{ height: "100%", textAlign: "left" }}
          >
            <Row style={{ height: "60px" }} align={"middle"}>
              <Col span={6}>
                <FormItem
                  {...slaveFormItemLayout}
                  label={<FormattedMessage id="x方向偏置" />}
                  initialValue={0}
                  name={"xoffset"}
                  rules={[
                    {
                      type: "number",
                      required: true,
                      message: <FormattedMessage id="请输入x方向偏置" />,
                    },
                  ]}
                >
                  <InputNumber style={{ width: "100%" }} size={"middle"} />
                </FormItem>
              </Col>
              <Col span={2}>
                <span style={{ marginLeft: "8px" }}>米</span>
              </Col>
              <Col span={6}>
                <FormItem
                  {...slaveFormItemLayout}
                  label={<FormattedMessage id="绕x旋转角度" />}
                  initialValue={0}
                  name={"rxoffset"}
                  rules={[
                    {
                      type: "number",
                      required: true,
                      message: <FormattedMessage id="请输入绕x旋转角度" />,
                    },
                  ]}
                >
                  <InputNumber style={{ width: "100%" }} />
                </FormItem>
              </Col>
              <Col span={2}>
                <span style={{ marginLeft: "8px" }}>度</span>
              </Col>
            </Row>

            <Row style={{ height: "60px" }} align={"middle"}>
              <Col span={6}>
                <FormItem
                  {...slaveFormItemLayout}
                  label={<FormattedMessage id="y方向偏置" />}
                  initialValue={0}
                  name={"yoffset"}
                  rules={[
                    {
                      type: "number",
                      required: true,
                      message: <FormattedMessage id="请输入y方向偏置" />,
                    },
                  ]}
                >
                  <InputNumber style={{ width: "100%" }} size={"middle"} />
                </FormItem>
              </Col>
              <Col span={2}>
                <span style={{ marginLeft: "8px" }}>米</span>
              </Col>
              <Col span={6}>
                <FormItem
                  {...slaveFormItemLayout}
                  label={<FormattedMessage id="绕y旋转角度" />}
                  initialValue={0}
                  name={"ryoffset"}
                  rules={[
                    {
                      type: "number",
                      required: true,
                      message: <FormattedMessage id="请输入绕y旋转角度" />,
                    },
                  ]}
                >
                  <InputNumber style={{ width: "100%" }} size={"middle"} />
                </FormItem>
              </Col>
              <Col span={2}>
                <span style={{ marginLeft: "8px" }}>度</span>
              </Col>
            </Row>

            <Row style={{ height: "60px" }} align={"middle"}>
              <Col span={6}>
                <FormItem
                  {...slaveFormItemLayout}
                  label={<FormattedMessage id="z方向偏置" />}
                  initialValue={0}
                  name={"zoffset"}
                  rules={[
                    {
                      type: "number",
                      required: true,
                      message: <FormattedMessage id="请输入z方向偏置" />,
                    },
                  ]}
                >
                  <InputNumber style={{ width: "100%" }} size={"middle"} />
                </FormItem>
              </Col>
              <Col span={2}>
                <span style={{ marginLeft: "8px" }}>米</span>
              </Col>
              <Col span={6}>
                <FormItem
                  {...slaveFormItemLayout}
                  label={<FormattedMessage id="绕z旋转角度" />}
                  initialValue={0}
                  name={"rzoffset"}
                  rules={[
                    {
                      type: "number",
                      required: true,
                      message: <FormattedMessage id="请输入绕z旋转角度" />,
                    },
                  ]}
                >
                  <InputNumber style={{ width: "100%" }} size={"middle"} />
                </FormItem>
              </Col>
              <Col span={2}>
                <span style={{ marginLeft: "8px" }}>度</span>
              </Col>
            </Row>
          </Form>
        )}
        <Button
          type={"danger"}
          disabled={
            ds || this.state.calibData[this.state.calibMethod].length === 0
          }
          onClick={() => {
            const data = { ...this.state.calibData };
            data[this.state.calibMethod] = [];
            this.setState({ calibData: data });
          }}
        >
          <FormattedMessage id="清空数据" />
        </Button>
        <Button
          type={"danger"}
          disabled={
            ds || this.state.calibData[this.state.calibMethod].length === 0
          }
          onClick={() => {
            const data = { ...this.state.calibData };
            data[this.state.calibMethod].pop();
            this.setState({ calibData: data });
          }}
        >
          <FormattedMessage id="删除单行数据" />
        </Button>
        <Button
          type={"primary"}
          disabled={
            ds ||
            !(
              endPE &&
              this.state.calibData[this.state.calibMethod].length <
                POINTS_COUNT[this.state.calibMethod]
            )
          }
          onClick={() => {
            const data = { ...this.state.calibData };
            data[this.state.calibMethod].push(endPE);
            this.setState({ calibData: data });
          }}
        >
          <FormattedMessage id="插入数据" />
        </Button>
        <Button
          type={"primary"}
          disabled={
            ds ||
            this.state.calibData[this.state.calibMethod].length !==
              POINTS_COUNT[this.state.calibMethod]
          }
          onClick={() => {
            switch (this.state.calibMethod) {
              case "CalibWobj":
                this.props.sendCmd(
                  `CalibW3P --tool=tool${this.props.currentToolId} --wobj=wobj${
                    this.state.toolId
                  } --pose={${this.state.calibData[this.state.calibMethod]
                    .join(",")
                    .replace(/ /g, ",")}}`,
                  (msg) => {
                    this.setState({ result: msg.jsData });
                  }
                );
                return;
              case "CalibFC":
                this.formRef.current.validateFields().then((values) => {
                  this.props.sendCmd(
                    `CalibFZero --xoffset=${values.xoffset} --rxoffset=${
                      values.rxoffset
                    } --yoffset=${values.yoffset} --ryoffset=${
                      values.ryoffset
                    } --zoffset=${values.zoffset} --rzoffset=${
                      values.rzoffset
                    } --pose={${this.state.calibData[this.state.calibMethod]
                      .join(",")
                      .replace(/ /g, ",")}}`,
                    (msg) => {
                      this.setState({ result: msg.jsData });
                    }
                  );
                });
                return;
              default:
                this.props.sendCmd(
                  `${this.state.calibMethod} --tool=tool${
                    this.state.toolId
                  } --pose={${this.state.calibData[this.state.calibMethod]
                    .join(",")
                    .replace(/ /g, ",")}}`,
                  (msg) => {
                    this.setState({ result: msg.jsData });
                  }
                );
            }
          }}
        >
          <FormattedMessage id="开始标定" />
        </Button>
        <Button
          type={"primary"}
          disabled={ds}
          onClick={() => {
            this.props.sendCmd("savexml");
          }}
        >
          <FormattedMessage id="保存标定结果" />
        </Button>
      </div>
    );
    return (
      <Table
        className={`${prefixCls}-table`}
        dataSource={data}
        columns={columns}
        size={"small"}
        locale={
          localestate === "chinese"
            ? {
                emptyText: "endPE数据载入中...",
              }
            : {
                emptyText: "endPE Data loading...",
              }
        }
        bordered
        pagination={false}
        footer={() => Footer}
      />
    );
  };

  getUserInputUpdateFn = (key) => {
    return (value) => {
      const userInput = { ...this.state.userInput };
      userInput[key] = value;
      this.setState({ userInput });
    };
  };

  getCellRenderer = (key) => {
    return (text, record) => {
      return (
        <InputNumber
          step={0.001}
          size={"small"}
          value={record[key]}
          onChange={this.getUserInputUpdateFn(key)}
        />
      );
    };
  };

  renderUserInput = (prefixCls) => {
    const { userInput } = this.state;
    const columns = [
      {
        title: "X[mm]",
        dataIndex: "x",
        key: "xi",
        align: "center",
        render: this.getCellRenderer("x"),
      },
      {
        title: "Y[mm]",
        dataIndex: "y",
        key: "yi",
        align: "center",
        render: this.getCellRenderer("y"),
      },
      {
        title: "Z[mm]",
        dataIndex: "z",
        key: "zi",
        align: "center",
        render: this.getCellRenderer("z"),
      },
      {
        title: "A",
        dataIndex: "a",
        key: "ai",
        align: "center",
        render: this.getCellRenderer("a"),
      },
      {
        title: "B",
        dataIndex: "b",
        key: "bi",
        align: "center",
        render: this.getCellRenderer("b"),
      },
      {
        title: "C",
        dataIndex: "c",
        key: "ci",
        align: "center",
        render: this.getCellRenderer("c"),
      },
    ];
    const data = [
      {
        xi: "xlast",
        yi: "ylast",
        zi: "zlast",
        ai: "alast",
        bi: "blast",
        ci: "clast",
        rowKey: "result",
        x: userInput.x,
        y: userInput.y,
        z: userInput.z,
        a: userInput.a,
        b: userInput.b,
        c: userInput.c,
        last: true,
      },
    ];
    const Footer = (
      <div className={`${prefixCls}-table-footer`}>
        <Button
          type={"primary"}
          onClick={() => {
            this.setState({
              showSaveResult: true,
            });

            this.props.sendCmd("savexml");
          }}
        >
          <FormattedMessage id="保存" />
        </Button>
      </div>
    );
    return (
      <div className={`${prefixCls}-calib-userinput`}>
        <Table
          className={`${prefixCls}-table`}
          dataSource={data}
          columns={columns}
          size={"small"}
          locale={{
            emptyText: "标定结果载入中...",
          }}
          bordered
          pagination={false}
          footer={() => Footer}
        />
      </div>
    );
  };

  // 力控标定返回结果
  renderFCResult = (prefixCls) => {
    const { showCalibResult } = this.state;

    if (!showCalibResult || !this.state.result) {
      return null;
    }
    if (this.state.result.return_code !== 0) {
      return (
        <div className={`${prefixCls}-calib-result error`}>
          <div>
            <FormattedMessage id="标定错误" />
          </div>
          <div className={"error"}>
            <FormattedMessage id="代码" />:{this.state.result.return_code}
          </div>
          <div className={"error"}>
            <FormattedMessage id="错误" />:{this.state.result.return_message}
          </div>
        </div>
      );
    }
    return (
      <div className={`${prefixCls}-calib-result-message`}>
        {this.state.result.CalibrationInfo}
      </div>
    );
  };

  renderWobjResult = (prefixCls) => {
    const { showCalibResult } = this.state;

    if (!showCalibResult || !this.state.result) {
      return null;
    }
    if (this.state.result.return_code !== 0 || !this.state.result.wobj_pe) {
      return (
        <div className={`${prefixCls}-calib-result error`}>
          <div>
            <FormattedMessage id="标定错误" />
          </div>
          <div className={"error"}>
            <FormattedMessage id="代码" />:{this.state.result.return_code}
          </div>
          <div className={"error"}>
            <FormattedMessage id="错误" />:{this.state.result.return_message}
          </div>
        </div>
      );
    }
    const columns = [
      {
        title: "X[mm]",
        dataIndex: "x",
        key: "xi",
        align: "center",
      },
      {
        title: "Y[mm]",
        dataIndex: "y",
        key: "yi",
        align: "center",
      },
      {
        title: "Z[mm]",
        dataIndex: "z",
        key: "zi",
        align: "center",
      },
      {
        title: "A",
        dataIndex: "a",
        key: "ai",
        align: "center",
      },
      {
        title: "B",
        dataIndex: "b",
        key: "bi",
        align: "center",
      },
      {
        title: "C",
        dataIndex: "c",
        key: "ci",
        align: "center",
      },
    ];
    const data = [
      {
        xi: "xlast",
        yi: "ylast",
        zi: "zlast",
        ai: "alast",
        bi: "blast",
        ci: "clast",
        rowKey: "result",
        x:
          this.state.result.wobj_pe &&
          (parseFloat(this.state.result.wobj_pe[0]) * 1000).toFixed(3),
        y:
          this.state.result.wobj_pe &&
          (parseFloat(this.state.result.wobj_pe[1]) * 1000).toFixed(3),
        z:
          this.state.result.wobj_pe &&
          (parseFloat(this.state.result.wobj_pe[2]) * 1000).toFixed(3),
        a:
          this.state.result.wobj_pe &&
          ((parseFloat(this.state.result.wobj_pe[3]) * 180) / Math.PI).toFixed(
            3
          ),
        b:
          this.state.result.wobj_pe &&
          ((parseFloat(this.state.result.wobj_pe[4]) * 180) / Math.PI).toFixed(
            3
          ),
        c:
          this.state.result.wobj_pe &&
          ((parseFloat(this.state.result.wobj_pe[5]) * 180) / Math.PI).toFixed(
            3
          ),
        last: true,
      },
    ];
    const Footer = (
      <div className={`${prefixCls}-table-footer`}>
        <Button
          type={"primary"}
          onClick={() => {
            this.setState({
              showCalibResult: false,
              showSaveResult: true,
            });
            this.props.sendCmd("savexml");
          }}
        >
          <FormattedMessage id="保存" />
        </Button>
      </div>
    );
    return (
      <div className={`${prefixCls}-calib-result`}>
        <div>
          <FormattedMessage id="标定结果" />
        </div>
        <Table
          className={`${prefixCls}-table`}
          dataSource={data}
          columns={columns}
          size={"small"}
          locale={{
            emptyText: "标定结果载入中...",
          }}
          bordered
          pagination={false}
          footer={() => Footer}
        />
        <div className={`${prefixCls}-calib-result-message`}>
          {this.state.result.CalibrationInfo}
        </div>
      </div>
    );
  };

  renderResult = (prefixCls) => {
    const { showCalibResult } = this.state;
    if (!showCalibResult || !this.state.result) {
      return null;
    }
    if (this.state.result.return_code !== 0 || !this.state.result.tool_pe) {
      return (
        <div className={`${prefixCls}-calib-result error`}>
          <div>
            <FormattedMessage id="标定错误" />
          </div>
          <div className={"error"}>
            <FormattedMessage id="代码" />:{this.state.result.return_code}
          </div>
          <div className={"error"}>
            <FormattedMessage id="错误" />:{this.state.result.return_message}
          </div>
        </div>
      );
    }
    const columns = [
      {
        title: "X[mm]",
        dataIndex: "x",
        key: "xi",
        align: "center",
      },
      {
        title: "Y[mm]",
        dataIndex: "y",
        key: "yi",
        align: "center",
      },
      {
        title: "Z[mm]",
        dataIndex: "z",
        key: "zi",
        align: "center",
      },
      {
        title: "A",
        dataIndex: "a",
        key: "ai",
        align: "center",
      },
      {
        title: "B",
        dataIndex: "b",
        key: "bi",
        align: "center",
      },
      {
        title: "C",
        dataIndex: "c",
        key: "ci",
        align: "center",
      },
    ];
    const data = [
      {
        xi: "xlast",
        yi: "ylast",
        zi: "zlast",
        ai: "alast",
        bi: "blast",
        ci: "clast",
        rowKey: "result",
        x:
          this.state.result.tool_pe &&
          (parseFloat(this.state.result.tool_pe[0]) * 1000).toFixed(3),
        y:
          this.state.result.tool_pe &&
          (parseFloat(this.state.result.tool_pe[1]) * 1000).toFixed(3),
        z:
          this.state.result.tool_pe &&
          (parseFloat(this.state.result.tool_pe[2]) * 1000).toFixed(3),
        a:
          this.state.result.tool_pe &&
          ((parseFloat(this.state.result.tool_pe[3]) * 180) / Math.PI).toFixed(
            3
          ),
        b:
          this.state.result.tool_pe &&
          ((parseFloat(this.state.result.tool_pe[4]) * 180) / Math.PI).toFixed(
            3
          ),
        c:
          this.state.result.tool_pe &&
          ((parseFloat(this.state.result.tool_pe[5]) * 180) / Math.PI).toFixed(
            3
          ),
        last: true,
      },
    ];
    const Footer = (
      <div className={`${prefixCls}-table-footer`}>
        <Button
          type={"primary"}
          onClick={() => {
            this.setState({
              showCalibResult: false,
              showSaveResult: true,
            });
            this.props.sendCmd("savexml");
          }}
        >
          <FormattedMessage id="保存" />
        </Button>
      </div>
    );
    return (
      <div className={`${prefixCls}-calib-result`}>
        <div>
          <FormattedMessage id="标定结果" />
        </div>
        <Table
          className={`${prefixCls}-table`}
          dataSource={data}
          columns={columns}
          size={"small"}
          locale={{
            emptyText: "标定结果载入中...",
          }}
          bordered
          pagination={false}
          footer={() => Footer}
        />
        <div className={`${prefixCls}-calib-result-message`}>
          {this.state.result.CalibrationInfo}
        </div>
      </div>
    );
  };

  renderSaveResult = (prefixCls) => {
    const { showSaveResult } = this.state;
    if (!showSaveResult || !this.state.result) {
      return null;
    }
    if (this.state.result.return_code !== 0) {
      return (
        <div className={`${prefixCls}-save-result error`}>
          <div>
            <FormattedMessage id="保存出错" />
          </div>
          <div className={"error"}>
            <FormattedMessage id="代码" />
            {this.state.result.return_code}
          </div>
          {!!this.state.result.CalibrationInfo && (
            <div className={"error"}>
              <FormattedMessage id="错误" />
              {this.state.result.CalibrationInfo}
            </div>
          )}
          {!!this.state.result.return_message && (
            <div className={"error"}>
              <FormattedMessage id="错误" />:{this.state.result.return_message}
            </div>
          )}
        </div>
      );
    } else {
      return (
        <div className={`${prefixCls}-save-result success`}>
          <div>
            <FormattedMessage id="保存成功" />
          </div>
          {!!this.state.result.CalibrationInfo && (
            <div className={`${prefixCls}-save-result-message success`}>
              {this.state.result.CalibrationInfo}
            </div>
          )}
          {!!this.state.result.return_message && (
            <div className={"success"}>{this.state.result.return_message}</div>
          )}
        </div>
      );
    }
  };

  renderContent = ({ getPrefixCls }) => {
    const {
      prefixCls: customizePrefixCls,
      toolIds,
      workObjectIds,
    } = this.props;
    const { calibMethod } = this.state;
    const prefixCls = getPrefixCls("calib-tool", customizePrefixCls);

    return (
      <div className={`${prefixCls}`}>
        <Row>
          <Col lg={10} className={`${prefixCls}-user-input-container`}>
            <div>
              <span>
                <FormattedMessage id="标定方法" />
              </span>
              <Select
                size={"small"}
                value={this.state.calibMethod}
                placeholder={<FormattedMessage id="选择标定方法" />}
                onChange={(calibMethod) => {
                  const toolId = this.state.toolId;
                  this.setState({
                    calibMethod,
                    pes:
                      JSON.parse(
                        window.localStorage.getItem(
                          `${calibMethod}-${toolId}-data`
                        )
                      ) || [],
                    showCalibResult: false,
                    toolName:
                      window.localStorage.getItem(
                        `${calibMethod}-${toolId}-name`
                      ) || null,
                    showSaveResult: false,
                  });
                  window.localStorage.setItem("calibMethod", calibMethod);
                }}
              >
                {Object.keys(CALIB_METHODS).map((name) => {
                  return (
                    <Option value={name} key={name}>
                      {<FormattedMessage id={CALIB_METHODS[name]} />}
                    </Option>
                  );
                })}
              </Select>
              <Popover
                placement="bottom"
                title={<span>{CALIB_METHODS[this.state.calibMethod]}</span>}
                content={DETAIL_HINTS[this.state.calibMethod]}
                trigger="click"
              >
                <QuestionOutlined />
              </Popover>
            </div>
            {(calibMethod === "UserInput" ||
              calibMethod === "CalibT4P" ||
              calibMethod === "CalibT5P" ||
              calibMethod === "CalibT6P") && (
              <div>
                <span>
                  <FormattedMessage id="工具号" />
                </span>
                <Select
                  size={"small"}
                  placeholder={<FormattedMessage id="选择工具号" />}
                  value={this.state.toolId}
                  onChange={this.handleSelectToolId}
                >
                  {toolIds.map((id, index) => {
                    return index > 0 ? (
                      <Option value={id} key={id}>
                        <FormattedMessage id="工具" />
                        {id}
                      </Option>
                    ) : null;
                  })}
                </Select>
              </div>
            )}
            {calibMethod === "CalibWobj" && (
              <div>
                <span>
                  <FormattedMessage id="工件号" />
                </span>
                <Select
                  size={"small"}
                  placeholder={<FormattedMessage id="选择工件号" />}
                  value={this.state.wobjId}
                  onChange={(e) => {
                    this.setState({ wobjId: e });
                  }}
                >
                  {workObjectIds.map((id, index) => {
                    return index > 0 ? (
                      <Option value={id} key={id}>
                        <FormattedMessage id="工件" />
                        {id}
                      </Option>
                    ) : null;
                  })}
                </Select>
              </div>
            )}
          </Col>
          <Col lg={14} className={`${prefixCls}-hints-container`}>
            {<FormattedMessage id={HINTS[this.state.calibMethod]} />}
          </Col>
        </Row>
        {this.state.calibMethod === "UserInput" && (
          <Row>{this.renderUserInput(prefixCls)}</Row>
        )}
        {this.state.calibMethod !== "UserInput" && (
          <Row> {this.renderTable(prefixCls)} </Row>
        )}
        {/* { this.state.calibMethod === 'CalibFC' && <Row> {this.renderTableFC(prefixCls)} </Row>} */}
        {this.state.calibMethod === "CalibFC" && (
          <Row> {this.renderFCResult(prefixCls)} </Row>
        )}
        {this.state.calibMethod === "CalibWobj" && (
          <Row> {this.renderWobjResult(prefixCls)} </Row>
        )}
        {this.state.calibMethod !== "UserInput" &&
          this.state.calibMethod !== "CalibFC" &&
          this.state.calibMethod !== "CalibWobj" && (
            <Row> {this.renderResult(prefixCls)} </Row>
          )}
        <Row>{this.renderSaveResult(prefixCls)}</Row>
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

ToolCalib.propTypes = {
  endPE: PropTypes.array,
  toolIds: PropTypes.array,
};

const getToolCalibState = createSelector(
  getMemoizeEndPE,
  getToolIds,
  getlocalestate,
  getWobjIds,
  getMemoizeMotionPos,
  getCurrentToolId,
  getStateCode,
  getWebsocketConnected,
  getMemoizeCsStarted,
  (
    endPE,
    toolIds,
    localestate,
    workObjectIds,
    motionPos,
    currentToolId,
    stateCode,
    isCsStarted,
    isWsConnected
  ) => {
    return {
      endPE,
      toolIds,
      localestate,
      workObjectIds,
      motionPos,
      currentToolId,
      stateCode,
      isCsStarted,
      isWsConnected,
    };
  }
);

function mapStateToProps(state, props) {
  return getToolCalibState(state, props);
}

const ConnectedToolCalib = connect(mapStateToProps, {
  sendCmd,
})(ToolCalib);

ConnectedToolCalib.NAME = "工具坐标系标定";
export default ConnectedToolCalib;
