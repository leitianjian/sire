import React from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import { connect } from "react-redux";
import { IntlProvider } from "react-intl";

import CellBase from "../CellBase";

// <Row>
// {this.renderHeader(prefixCls)}
// </Row>
// <Row>
// {this.renderTable(prefixCls)}
// </Row>
// <Row>
// {this.renderFooter(prefixCls)}
// </Row>
// import { Transfer } from 'antd/lib/transfer';
import { Transfer } from "antd";

import "antd/lib/transfer/style";
// import 'antd/lib/tree/style'
// import 'antd/lib/tree-select/style'
// import 'antd/dist/antd.less'

// Customize Table Transfer
// const isChecked = (selectedKeys, eventKey) => {
//   return selectedKeys.indexOf(eventKey) !== -1;
// };

// const generateTree = (treeNodes = [], checkedKeys = []) => {
//   console.log("generate");

//   return treeNodes.map(({ children, ...props }) => ({
//     ...props,
//     disabled: checkedKeys.includes(props.key),
//     children: generateTree(children, checkedKeys),
//   }));
// };

// const treeData = [
//   { key: "0-0", title: "0-0" },
//   { key: "0-1", title: "0-2" },
//   { key: "0-2", title: "0-3" },
//   { key: "0-3", title: "0-0" },
//   { key: "0-4", title: "0-2" },
//   { key: "0-5", title: "0-3" },
// ];

class ProgramLib extends CellBase {
  handleChange = (nextTargetKeys, direction, moveKeys) => {
    this.updateOptions({
      ...this.props.cell.options,
      targetKeys: nextTargetKeys,
    });
  };

  renderTree = (prefixCls) => {
    const dataSource = [];

    if (Object.keys(this.props.programs).length > 0) {
      Object.keys(this.props.programs).forEach((program_key) => {
        const program = this.props.programs[program_key];
        program.files.forEach((file) => {
          const name = program.name + "." + file.name.split(".")[0];
          if (
            dataSource.find((value) => {
              return value.key === name;
            }) === undefined
          ) {
            dataSource.push({ key: name });
          }
        });
      });
    }

    const options = this.getOptions();

    return (
      <Transfer
        className={`${prefixCls}-transfer`}
        targetKeys={options.targetKeys}
        dataSource={dataSource}
        render={(item) => item.key}
        onChange={this.handleChange}
      ></Transfer>
    );
  };

  renderContent = ({ getPrefixCls }) => {
    const prefixCls = getPrefixCls("prolib", this.props.prefixCls);

    return <div className={`${prefixCls}`}>{this.renderTree(prefixCls)}</div>;
  };

  render() {
    return (
      <IntlProvider>
        <ConfigConsumer>{this.renderContent}</ConfigConsumer>
      </IntlProvider>
    );
  }
}

function mapStateToProps(state) {
  return {
    programs: state.robot.programs,
  };
}

function mapDispatchToProps(dispatch) {
  return {
    // createProgram                 :(...args) => dispatch(createProgram            (...args)),
  };
}

const ConnectedProgramLib = connect(
  mapStateToProps,
  mapDispatchToProps
)(ProgramLib);

ConnectedProgramLib.NAME = "导入程序";
ConnectedProgramLib.WIDTH = 24;
ConnectedProgramLib.HEIGHT = 12;
export default ConnectedProgramLib;
