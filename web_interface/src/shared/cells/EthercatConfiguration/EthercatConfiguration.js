import React, { Fragment } from "react";
import CellBase from "../CellBase";
import isEqual from "lodash/isEqual";
import { connect } from "react-redux";
import Modal from "antd/lib/modal";
import Select from "antd/lib/select";

import { createSelector } from "reselect";
import Row from "antd/lib/row";
import Col from "antd/lib/col";
import Form from "antd/lib/form";
import Checkbox from "antd/lib/checkbox";
import Input from "antd/lib/input";
import InputNumber from "antd/lib/input-number";

import Button from "antd/lib/button";
import { CloseOutlined } from "@ant-design/icons";

import { loadEsiPath, loadConfigXml, sendCmd } from "../../../state/actions";
import {
  getlocalestate,
  getMemoizeCsStarted,
  getWebsocketConnected,
} from "../../../state/selectors";
import { getScanSlaveJsonResult } from "./selectors";

import EthercatController from "../../../ethercat/EthercatController";

// 进行中英文切换
import zhCN from "../locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "../locale/enUS";

const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

const Option = Select.Option;
const FormItem = Form.Item;

const slaveFormItemLayout = {
  labelCol: {
    span: 10,
  },
  wrapperCol: {
    span: 14,
  },
};

const hexValidator = (rule, value, callback) => {
  if (typeof value === "string" && !!value.match(/^(?:0[xX])?[0-9a-fA-F]+$/)) {
    callback();
  } else {
    // eslint-disable-next-line n/no-callback-literal
    callback("not hex");
  }
};

const decimalValidator = (rule, value, callback) => {
  callback();
};

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

class EthercatConfiguration extends CellBase {
  name = "Ethercat从站配置";
  width = 24;
  height = 24;
  state = {
    selectedSlave: null,
    saveState: true,
    showUpdateModal: false,
    beforeSaveModel: false,
    slaves: [],
  };

  formRef = React.createRef();

  getEsiPdoListForDevice(vendorId, productCode, revisionNum) {
    this.props.sendCmd(
      `getesipdolist --vendor_id=${vendorId} --product_code=${productCode} --revision_num=${revisionNum}`,
      (msg) => {}
    );
  }

  getEsiPdoListForSlave(vendorId, productCode, revisionNum) {
    this.props.sendCmd(
      `getesipdolist --vendor_id=${vendorId} --product_code=${productCode} --revision_num=${revisionNum}`,
      (msg) => {}
    );
  }

  selectSlave = (index) => {
    const { selectedSlave } = this.state;
    if (selectedSlave) {
      this.formRef.current
        .validateFields()
        .then((values) => {
          // 获取表单中的值
          const slaves = [...this.state.slaves];
          slaves[this.state.selectedSlave] = { ...values.slave };
          this.setState({
            slaves,
            selectedSlave: index,
            updateError: null,
          });
        })
        .catch((e) => {
          this.setState({ showUpdateModal: index });
        });
    } else {
      this.setState({ selectedSlave: index, updateError: null });
    }
  };

  addSlave = () => {
    const defaultSlave = {
      phyId: 0,
      dcAssignActivate: "0x00000000",
      isMotion: false,
      isVirtual: null,
      productCode: "0x00000000",
      revisionNum: "0x00000000",
      sync0ShiftNs: "650000",
      syncManagerPoolObjects: [{ syncManagers: [] }],
      vendorId: "0x00000000",
    };

    const newSlave = this.state.selectedSlave
      ? { ...this.state.slaves[this.state.selectedSlave] }
      : { ...defaultSlave };
    newSlave.uuid = Math.random().toString(36);
    const slaves = [...this.state.slaves];
    slaves.push(newSlave);
    this.setState({ slaves, updateError: null });
  };

  removeSlave = (index) => {
    const slaves = [...this.state.slaves];
    slaves.splice(index, 1);
    this.setState({ slaves, selectedSlave: null });
  };

  addSm = () => {
    const slaves = [...this.state.slaves];
    const sm = { isTx: "pdo", pdoes: [] };
    slaves[
      this.state.selectedSlave
    ].syncManagerPoolObjects[0].syncManagers.push(sm);
    this.setState({ slaves, updateError: null });
  };

  removeSm = (syncManagerIndex) => {
    const slave = { ...this.formRef.current.getFieldsValue().slave };
    slave.syncManagerPoolObjects[0].syncManagers.splice(syncManagerIndex, 1);
    this.formRef.current.setFieldsValue({ slave });
    const slaves = [...this.state.slaves];
    slaves[this.state.selectedSlave] = { ...slave };
    this.setState({ slaves, updateError: null });
  };

  addPdo = (syncManagerIndex) => {
    const slaves = [...this.state.slaves];
    const pdo = { name: "pdo", index: "0x0000", entries: [] };
    slaves[this.state.selectedSlave].syncManagerPoolObjects[0].syncManagers[
      syncManagerIndex
    ].pdoes.push(pdo);
    this.setState({ slaves, updateError: null });
  };

  removePdo = (syncManagerIndex, pdoIndex) => {
    const slave = { ...this.formRef.current.getFieldsValue().slave };
    slave.syncManagerPoolObjects[0].syncManagers[syncManagerIndex].pdoes.splice(
      pdoIndex,
      1
    );
    this.formRef.current.setFieldsValue({ slave });
    const slaves = [...this.state.slaves];
    slaves[this.state.selectedSlave] = { ...slave };
    this.setState({ slaves, updateError: null });
  };

  addPdoEntry = (syncManagerIndex, pdoIndex) => {
    const slaves = [...this.state.slaves];
    const entry = {
      name: "entry",
      index: "0x0000",
      subIndex: "0x00",
      size: "32",
    };
    slaves[this.state.selectedSlave].syncManagerPoolObjects[0].syncManagers[
      syncManagerIndex
    ].pdoes[pdoIndex].entries.push(entry);
    this.setState({ slaves, updateError: null });
  };

  removePdoEntry = (syncManagerIndex, pdoIndex, pdoEntryIndex) => {
    const slave = { ...this.formRef.current.getFieldsValue().slave };
    slave.syncManagerPoolObjects[0].syncManagers[syncManagerIndex].pdoes[
      pdoIndex
    ].entries.splice(pdoEntryIndex, 1);
    this.formRef.current.setFieldsValue({ slave });
    const slaves = [...this.state.slaves];
    slaves[this.state.selectedSlave] = { ...slave };
    this.setState({ slaves, updateError: null });
  };

  saveSlaves = (e) => {
    // 保存slaves的数据
    e.preventDefault();

    if (this.state.selectedSlave) {
      this.formRef.current
        .validateFields()
        .then((values) => {
          // 获取表单中的值
          // update slaves //
          const slaves = [...this.state.slaves];
          slaves[this.state.selectedSlave] = { ...values.slave };
          this.setState({ slaves, updateError: null });

          // make json file //
          const ethercatController = {
            name: "ethercat_controller",
            slavePoolObjects: [
              {
                name: "slave_pool",
                slaves: [...slaves],
              },
            ],
          };

          const ec = EthercatController.FromJSON(ethercatController);

          // send to server //
          this.props.sendCmd(
            `set_xml --xml={<ControlServer>${ec.toXML()}</ControlServer>}`
          );
          this.props.sendCmd("savexml");
        })
        .catch((e) => {
          console.log(e);
          this.setState({ beforeSaveModel: true });
        });
    } else {
      // make json file //
      const ethercatController = {
        name: "ethercat_controller",
        slavePoolObjects: [
          {
            name: "slave_pool",
            slaves: [...this.state.slaves],
          },
        ],
      };
      const ec = EthercatController.FromJSON(ethercatController);

      // send to server //
      this.props.sendCmd(
        `set_xml --xml={<ControlServer>${ec.toXML()}</ControlServer>}`
      );
      this.props.sendCmd("savexml");
    }
  };

  handleUpdateOk = () => {
    this.setState({
      updateError: "请修正不合理的输入内容!",
      showUpdateModal: false,
    });
  };

  handleUpdateCancel = () => {
    const index = this.state.showUpdateModal;
    this.setState({
      showUpdateModal: false,
      selectedSlave: index,
      updateError: null,
    });
  };

  handleBeforeSaveUpdateOk = () => {
    this.setState({
      updateError: "请修正不合理的输入内容!",
      beforeSaveModel: false,
    });
  };

  handleBeforeSaveUpdateCancel = () => {
    this.setState({ beforeSaveModel: false });
  };

  shouldComponentUpdate(nextProps, nextState, nextContext) {
    if (!isEqual(nextProps.ethercatController, this.props.ethercatController)) {
      if (
        nextProps.ethercatController &&
        nextProps.ethercatController.slavePoolObjects &&
        nextProps.ethercatController.slavePoolObjects[0]
      ) {
        const slaves = []; // 定义一个slaves对象
        nextProps.ethercatController.slavePoolObjects[0].slaves &&
          nextProps.ethercatController.slavePoolObjects[0].slaves.forEach(
            (m) => {
              slaves.push(m);
            }
          );
        this.setState({ slaves, updateError: null });
      }
      return false;
    }
    return true;
  }

  componentDidUpdate(prevProps, prevState) {
    if (this.state.selectedSlave !== prevState.selectedSlave) {
      if (this.formRef.current) {
        this.formRef.current.resetFields();
      }
    }
  }

  componentDidMount() {
    this.props.loadEsiPath();
    this.props.loadConfigXml();

    if (this.props.ethercatController) {
      if (
        this.props.ethercatController &&
        this.props.ethercatController.slavePoolObjects &&
        this.props.ethercatController.slavePoolObjects[0]
      ) {
        const slaves = []; // 定义一个slaves对象
        this.props.ethercatController.slavePoolObjects[0].slaves &&
          this.props.ethercatController.slavePoolObjects[0].slaves.forEach(
            (m) => {
              slaves.push(m);
            }
          );
        this.setState({ slaves, updateError: null });
      }
    }
  }

  renderHeader = (prefixCls) => {
    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    return (
      <div className={`${prefixCls}-header`}>
        <Button
          disabled={!this.props.isWsConnected || !this.props.isCsStarted}
          type={"primary"}
          className={`${prefixCls}-header-buttons`}
          onClick={() => this.props.sendCmd("cs_stop")}
        >
          开始配置
        </Button>
        <Button
          disabled={!this.props.isWsConnected || !!this.props.isCsStarted}
          type={"primary"}
          className={`${prefixCls}-header-buttons`}
          onClick={() => this.props.sendCmd("cs_start")}
        >
          结束配置
        </Button>
        <Button
          disabled={ds}
          type={"primary"}
          className={`${prefixCls}-header-buttons`}
          onClick={this.saveSlaves}
        >
          保存配置
        </Button>
        <Button
          disabled={ds}
          type={"primary"}
          className={`${prefixCls}-header-buttons`}
          onClick={this.addSlave}
        >
          添加从站
        </Button>
        <Button
          disabled={ds}
          type={"primary"}
          className={`${prefixCls}-header-buttons`}
          onClick={() => {
            this.props.sendCmd("scanslave", (msg) => {
              if (msg.jsData.controller_xml) {
                const parser = new DOMParser();
                const xmlDoc = parser.parseFromString(
                  msg.jsData.controller_xml,
                  "text/xml"
                );
                const masterNode =
                  xmlDoc.getElementsByTagName("EthercatController"); // 获取到EthercatController节点
                const ec = EthercatController.FromXML(masterNode[0]).toJSON();

                if (ec && ec.slavePoolObjects && ec.slavePoolObjects[0]) {
                  const slaves = []; // 定义一个slaves对象
                  ec.slavePoolObjects[0].slaves &&
                    ec.slavePoolObjects[0].slaves.forEach((m) => {
                      slaves.push(m);
                    });
                  this.setState({ slaves, updateError: null });
                }
              }
            });
          }}
        >
          扫描从站
        </Button>
      </div>
    );
  };

  renderLeftSider = (prefixCls) => {
    const slaves = [...this.state.slaves];
    const selectedSlave = this.state.selectedSlave;

    return (
      <div className={`${prefixCls}-sidebar`}>
        <div className={`${prefixCls}-slaves-list`}>
          {Object.keys(slaves).map((key) => {
            return (
              <div
                key={key}
                className={`${
                  selectedSlave === key ? `${prefixCls}-selected-slave` : ""
                } ${prefixCls}-slaves-list-item`}
                onClick={() => this.selectSlave(key)}
              >
                {slaves[key].isMotion ? (
                  <FormattedMessage id={`${key}:电机`} />
                ) : (
                  <FormattedMessage id={`${key}:从站`} />
                )}
              </div>
            );
          })}
        </div>
      </div>
    );
  };

  renderSlaveInfo = (prefixCls) => {
    const selectedSlave = this.state.selectedSlave;
    if (!selectedSlave) return <div />;

    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    const slaves = this.state.slaves;
    const slave = slaves[selectedSlave];
    return (
      <div className={`${prefixCls}-content`} onSubmit={() => {}}>
        <Form ref={this.formRef}>
          <div className={`${prefixCls}-content-header`}>
            <FormItem
              name={["slave", "isVirtual"]}
              initialValue={slave.isVirtual}
              valuePropName="checked"
              rules={[]}
            >
              <Checkbox
                disabled={ds}
                checked={slave.isVirtual}
                onChange={(e) => {
                  const newSlaves = [...this.state.slaves];
                  newSlaves[selectedSlave] = {
                    ...newSlaves[selectedSlave],
                    isVirtual: e.target.checked,
                  };
                  this.setState({ slaves: newSlaves, updateError: null });
                }}
              >
                Is Virtual
              </Checkbox>
            </FormItem>
            <FormItem
              name={["slave", "isMotion"]}
              initialValue={slave.isMotion}
              valuePropName="checked"
              rules={[]}
            >
              <Checkbox
                disabled={ds}
                checked={slave.isMotion}
                onChange={(e) => {
                  const newSlaves = [...this.state.slaves];
                  newSlaves[selectedSlave] = {
                    ...newSlaves[selectedSlave],
                    isMotion: e.target.checked,
                  };
                  this.setState({ slaves: newSlaves, updateError: null });
                }}
              >
                Is Motion
              </Checkbox>
            </FormItem>
            <Button
              style={{ float: "right" }}
              disabled={ds}
              size={"small"}
              type="primary"
              shape="circle"
              icon={<CloseOutlined />}
              onClick={() => {
                this.removeSlave(this.state.selectedSlave);
              }}
            />
          </div>
          {!!slave.isMotion && this.renderMotionInputs(prefixCls, slave)}
          {!slave.isMotion && this.renderSlaveInputs(prefixCls, slave)}
          {!!slave &&
            slave.syncManagerPoolObjects &&
            slave.syncManagerPoolObjects[0] &&
            slave.syncManagerPoolObjects[0].syncManagers &&
            slave.syncManagerPoolObjects[0].syncManagers.map((sm, index) =>
              this.renderSyncManager(prefixCls, sm, index)
            )}
          <div style={{ height: "32px" }}>
            <Button
              disabled={ds}
              size={"small"}
              type="primary"
              className={`${prefixCls}-content-sm-add`}
              onClick={() => {
                this.addSm();
              }}
            >
              添加SM
            </Button>
          </div>
        </Form>
      </div>
    );
  };

  renderFormItemInput(
    prefixCls,
    key,
    label,
    initialValue,
    rules,
    span,
    required
  ) {
    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    return (
      <Col span={span || 7} offset={1} pull={1} key={key}>
        <FormItem
          {...slaveFormItemLayout}
          label={label}
          hasFeedback
          name={["slave", `${key}`]}
          initialValue={initialValue}
          rules={[
            {
              required: !!required,
              message: `Please input ${label}`,
            },
          ]}
        >
          <Input disabled={ds} placeholder={`${label}`} size={"small"} />
        </FormItem>
      </Col>
    );
  }

  renderFormItemNumberInput(
    prefixCls,
    key,
    label,
    initialValue,
    span,
    required,
    step,
    postfix,
    min,
    max,
    unit,
    unitName
  ) {
    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    if (key === "maxVel" || key === "maxAcc") {
      return (
        <Col span={span || 7} offset={1} pull={1} key={key}>
          <FormItem
            {...slaveFormItemLayout}
            label={label}
            hasFeedback
            name={["slave", `${key}`]}
            initialValue={initialValue}
            rules={[
              {
                required: !!required,
                message: `Please input ${label}`,
              },
            ]}
          >
            <InputNumber
              disabled={ds}
              size={"small"}
              step={step || 0.01}
              min={min || -Infinity}
              max={max || Infinity}
              placeholder={`${label}`}
              onChange={(e) => {
                const value = -Number(e);
                if (key === "maxVel") {
                  this.formRef.current.setFieldsValue({
                    "slave.minVel": value,
                  });
                }
                if (key === "maxAcc") {
                  this.formRef.current.setFieldsValue({
                    "slave.minAcc": value,
                  });
                }
              }}
            />
          </FormItem>
          {!!postfix && (
            <div className={`${prefixCls}-form-postfix`}>{postfix}</div>
          )}
        </Col>
      );
    }
    return (
      <Col span={span || 7} offset={1} pull={1} key={key}>
        <FormItem
          {...slaveFormItemLayout}
          label={label}
          hasFeedback
          name={["slave", `${key}`]}
          initialValue={initialValue}
          rules={[
            {
              required: !!required,
              message: `Please input ${label}`,
            },
          ]}
        >
          <InputNumber
            disabled={ds}
            size={"small"}
            step={step || 0.01}
            min={min || -Infinity}
            max={max || Infinity}
            placeholder={`${label}`}
          />
        </FormItem>
        {!!postfix && (
          <div className={`${prefixCls}-form-postfix`}>{postfix}</div>
        )}
      </Col>
    );
  }

  renderSlaveInputs = (prefixCls, slave) => {
    return (
      <Row>
        {[
          [
            "phyId",
            "Physical ID",
            slave.phyId,
            [{ type: "integer", message: "Please input integer" }],
          ],
          [
            "productCode",
            "ProductCode",
            slave.productCode,
            [{ validator: hexValidator, message: "Please input hex" }],
          ],
          [
            "sync0ShiftNs",
            "Sync0 Shift ns",
            slave.sync0ShiftNs,
            [{ type: "integer", message: "Please input integer" }],
          ],
          [
            "vendorId",
            "Vendor ID",
            slave.vendorId,
            [{ validator: hexValidator, message: "Please input hex" }],
          ],
          [
            "revisionNum",
            "Revision No.",
            slave.revisionNum,
            [{ validator: hexValidator, message: "Please input hex" }],
          ],
          [
            "dcAssignActivate",
            "Dc Assign Activate",
            slave.dcAssignActivate,
            [
              {
                validator: hexValidator,
                message: "Please input hex",
              },
            ],
          ],
        ].map((d) => this.renderFormItemInput(prefixCls, ...d, 7, true))}
      </Row>
    );
  };

  renderMotionInputs = (prefixCls, slave) => {
    return (
      <Fragment>
        {this.renderSlaveInputs(prefixCls, slave)}
        <Row>
          {[
            [
              "maxPos",
              "Max Pos",
              slave.maxPos,
              7,
              true,
              0.01,
              " °(m)",
              -Infinity,
              Infinity,
              slave.maxPosPostfix,
              "maxPosPostfix",
            ],
            [
              "maxVel",
              "Max Vel",
              slave.maxVel,
              7,
              true,
              0.01,
              " °(m)/s",
              -Infinity,
              Infinity,
              slave.maxVelPostfix,
              "maxVelPostfix",
            ],
            [
              "maxAcc",
              "Max Acc",
              slave.maxAcc,
              7,
              true,
              0.01,
              " °(m)/s2",
              -Infinity,
              Infinity,
              slave.maxAccPostfix,
              "maxAccPostfix",
            ],
            [
              "minPos",
              "Min Pos",
              slave.minPos,
              7,
              true,
              0.01,
              " °(m)",
              -Infinity,
              Infinity,
              slave.minPosPostfix,
              "minPosPostfix",
            ],
            [
              "minVel",
              "Min Vel",
              slave.minVel,
              7,
              true,
              0.01,
              " °(m)/s",
              -Infinity,
              Infinity,
              slave.minVelPostfix,
              "minVelPostfix",
            ],
            [
              "minAcc",
              "Min Acc",
              slave.minAcc,
              7,
              true,
              0.01,
              " °(m)/s2",
              -Infinity,
              Infinity,
              slave.minAccPostfix,
              "minAccPostfix",
            ],
            [
              "posFactor",
              "Pos Factor",
              slave.posFactor,
              7,
              true,
              0.01,
              "",
              -Number.MAX_VALUE,
              Number.MAX_VALUE,
            ],
            [
              "posOffset",
              "Pos Offset",
              slave.posOffset,
              7,
              true,
              0.01,
              " °(m)",
              -Infinity,
              Infinity,
              slave.posOffsetPostfix,
              "posOffsetPostfix",
            ],
            [
              "homePos",
              "Home Pos",
              slave.homePos,
              7,
              true,
              0.01,
              " °(m)",
              -Infinity,
              Infinity,
              slave.homePosPostfix,
              "homePosPostfix",
            ],
            [
              "maxPosFollowingError",
              "Max Pos Following Error",
              slave.maxPosFollowingError,
              11,
              true,
              0.0001,
              "",
              -Number.MAX_VALUE,
              Number.MAX_VALUE,
            ],
            [
              "maxVelFollowingError",
              "Max Vel Following Error",
              slave.maxVelFollowingError,
              11,
              true,
              0.0001,
              "",
              -Number.MAX_VALUE,
              Number.MAX_VALUE,
            ],
          ].map((d) => this.renderFormItemNumberInput(prefixCls, ...d))}
        </Row>
      </Fragment>
    );
  };

  renderPdoEntry = (
    prefixCls,
    pdoEntry,
    syncMangerIndex,
    pdoIndex,
    pdoEntryIndex
  ) => {
    const rules = {
      name: [],
      index: [
        {
          required: true,
          validator: hexValidator,
          message: "Please input hex",
        },
      ],
      subIndex: [
        {
          required: true,
          validator: decimalValidator,
          message: "Please input decimal",
        },
      ],
      size: [
        {
          required: true,
          type: "integer",
          message: "Please input integer",
          transform: (value) => {
            return Number(value) ? Number(value) : 0;
          },
        },
      ],
    };
    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    return (
      <Row key={pdoEntryIndex}>
        <Col span={4}>Entry</Col>
        {["name", "index", "subIndex", "size"].map((key) => (
          <Col key={key} span={4}>
            <FormItem
              hasFeedback={true}
              name={[
                "slave",
                "syncManagerPoolObjects",
                0,
                "syncManagers",
                syncMangerIndex,
                "pdoes",
                pdoIndex,
                "entries",
                pdoEntryIndex,
                `${key}`,
              ]}
              initialValue={pdoEntry[key]}
              rules={rules[key]}
            >
              <Input disabled={ds} placeholder={`${key}`} size={"small"} />
            </FormItem>
          </Col>
        ))}
        <Col span={4} style={{ textAlign: "right" }}>
          <Button
            disabled={ds}
            size={"small"}
            type="primary"
            shape="circle"
            icon={<CloseOutlined />}
            onClick={() => {
              this.removePdoEntry(syncMangerIndex, pdoIndex, pdoEntryIndex);
            }}
          />
        </Col>
      </Row>
    );
  };

  renderPdo = (prefixCls, pdo, syncMangerIndex, pdoIndex) => {
    const rules = {
      index: [
        {
          required: true,
          validator: hexValidator,
          message: "Please input hex",
        },
      ],
    };
    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    return (
      <div className={`${prefixCls}-content-sm-pdo`} key={pdoIndex}>
        <Row className={`${prefixCls}-content-sm-pdo-header`}>
          <Col span={2}>Pdo</Col>
          {["index"].map((key) => (
            <Col span={6} key={key}>
              <FormItem
                {...slaveFormItemLayout}
                label={key}
                hasFeedback={true}
                name={[
                  "slave",
                  "syncManagerPoolObjects",
                  0,
                  "syncManagers",
                  syncMangerIndex,
                  "pdoes",
                  pdoIndex,
                  `${key}`,
                ]}
                initialValue={pdo[key]}
                rules={rules[key]}
              >
                <Input disabled={ds} placeholder={`${key}`} size={"small"} />
              </FormItem>
            </Col>
          ))}
          <Col span={16} style={{ textAlign: "right" }}>
            <Button
              disabled={ds}
              size={"small"}
              type="primary"
              shape="circle"
              icon={<CloseOutlined />}
              onClick={() => {
                this.removePdo(syncMangerIndex, pdoIndex);
              }}
            />
          </Col>
        </Row>
        <div className={`${prefixCls}-content-sm-pdo-entry`}>
          <Row className={`${prefixCls}-content-sm-pdo-entry-header`}>
            <Col span={4} />
            {["name", "index", "subIndex", "size"].map((key) => (
              <Col key={key} span={4}>
                {key}
              </Col>
            ))}
          </Row>
          {!!pdo.entries &&
            pdo.entries.map((pdoEntry, pdoEntryIndex) =>
              this.renderPdoEntry(
                prefixCls,
                pdoEntry,
                syncMangerIndex,
                pdoIndex,
                pdoEntryIndex
              )
            )}
        </div>
        <div style={{ height: "32px", display: "block" }}>
          <Button
            disabled={ds}
            size={"small"}
            type={"primary"}
            className={`${prefixCls}-content-sm-pdo-entry-add`}
            onClick={() => {
              this.addPdoEntry(syncMangerIndex, pdoIndex);
            }}
          >
            添加Entry
          </Button>
        </div>
      </div>
    );
  };

  renderSyncManager = (prefixCls, syncManager, syncManagerIndex) => {
    const ds = !this.props.isWsConnected || !!this.props.isCsStarted;
    return (
      <div className={`${prefixCls}-content-sm`} key={syncManagerIndex}>
        <div className={`${prefixCls}-content-sm-header`}>
          Sync Manager {syncManagerIndex + 1}(SM{syncManagerIndex + 1})
          <Form.Item
            name={[
              "slave",
              "syncManagerPoolObjects",
              0,
              "syncManagers",
              syncManagerIndex,
              "isTx",
            ]}
            initialValue={syncManager.isTx}
          >
            <Select
              disabled={ds}
              size="small"
              style={{ width: 100, marginLeft: 5 }}
            >
              <Option value={"true"} key={"true"}>
                is_tx
              </Option>
              <Option value={"false"} key={"false"}>
                is_rx
              </Option>
            </Select>
          </Form.Item>
          <Button
            style={{ float: "right" }}
            disabled={ds}
            size={"small"}
            type="primary"
            shape="circle"
            icon={<CloseOutlined />}
            onClick={() => {
              this.removeSm(syncManagerIndex);
            }}
          />
        </div>
        <div className={`${prefixCls}-content-sm-pdo`}>
          {!!syncManager.pdoes &&
            syncManager.pdoes.map((pdo, pdoIndex) =>
              this.renderPdo(prefixCls, pdo, syncManagerIndex, pdoIndex)
            )}
        </div>
        <div style={{ display: "block", height: "32px" }}>
          <Button
            style={{ float: "right" }}
            disabled={ds}
            size={"small"}
            type={"primary"}
            className={`${prefixCls}-content-sm-pdo-add`}
            onClick={this.addPdo.bind(this, syncManagerIndex)}
          >
            添加PDO
          </Button>
        </div>
      </div>
    );
  };

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, localestate } = this.props;

    const prefixCls = getPrefixCls(
      "ethercat-configuration",
      customizePrefixCls
    );
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <div className={prefixCls}>
          <Row>{this.renderHeader(prefixCls)}</Row>
          {this.renderLeftSider(prefixCls)}
          {this.renderSlaveInfo(prefixCls)}
          <Modal
            title="提示"
            visible={this.state.showUpdateModal !== false}
            onOk={this.handleUpdateOk}
            onCancel={this.handleUpdateCancel}
            okText={"继续修改"}
            cancelText={"强制切换"}
            footer={[
              <Button key="change" type="primary" onClick={this.handleUpdateOk}>
                继续修改
              </Button>,
              <Button
                key="switch"
                type="danger"
                onClick={this.handleUpdateCancel}
              >
                强制切换
              </Button>,
            ]}
          >
            <p>
              当前表单的输入信息有误，无法自动保存。
              如果强制切换Slave，当前录入数据将丢失!
            </p>
          </Modal>
          <Modal
            title="提示"
            visible={this.state.beforeSaveModel !== false}
            onOk={this.handleBeforeSaveUpdateOk}
            onCancel={this.handleBeforeSaveUpdateCancel}
            okText={"继续修改"}
            cancelText={"强制保存"}
            footer={[
              <Button
                key="change"
                type="primary"
                onClick={this.handleBeforeSaveUpdateOk}
              >
                继续修改
              </Button>,
              <Button
                key="switch"
                type="danger"
                onClick={this.handleBeforeSaveUpdateCancel}
              >
                强制保存
              </Button>,
            ]}
          >
            <p>
              当前表单的输入信息有误，无法保存xml。 如果强制保存，可能引发错误!
            </p>
          </Modal>
        </div>
      </IntlProvider>
    );
  };
}

const getModelConfigurationState = createSelector(
  getScanSlaveJsonResult,
  getlocalestate,
  getMemoizeCsStarted,
  getWebsocketConnected,
  (ethercatController, localestate, isCsStarted, isWsConnected) => {
    return {
      ethercatController,
      localestate,
      isCsStarted,
      isWsConnected,
    };
  }
);

function mapStateToProps(state, props) {
  return getModelConfigurationState(state, props);
}

export default connect(mapStateToProps, {
  loadEsiPath,
  loadConfigXml,
  sendCmd,
})(EthercatConfiguration);
