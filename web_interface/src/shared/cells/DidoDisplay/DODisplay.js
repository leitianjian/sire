import React from "react";
import { connect } from "react-redux";
import CellBase from "../CellBase";
import Row from "antd/lib/row";
import { BulbFilled } from "@ant-design/icons";
import { getMemoizeDO } from "../../../state/selectors";

import { Typography } from "antd";
const { Paragraph } = Typography;

class DODisplay extends CellBase {
  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, DO } = this.props;
    const prefixCls = getPrefixCls("dido-display", customizePrefixCls);

    const options = this.getOptions();

    const is_di_same_length = (di1, di2) => {
      let i;
      for (i = 0; i < di1.length; ++i) {
        if (di1[i].length !== di2[i].length) return false;
      }

      return true;
    };

    // 重新构造序列 //
    if (!DO) {
    } else if (
      !!options.DO &&
      options.DO.length === DO.length &&
      is_di_same_length(options.DO, DO)
    ) {
      const newOpt = { ...options };
      let i, j;
      let has_changed = false;
      for (i = 0; i < DO.length; ++i) {
        for (j = 0; j < DO[i].length; ++j) {
          if (newOpt.DO[i][j].value !== DO[i][j]) {
            newOpt.DO[i][j].value = DO[i][j];
            has_changed = true;
          }
        }
      }

      if (has_changed) this.updateOptions(newOpt);
    } else {
      const new_di = options.DO ? options.DO : [];

      // 删除多余模块 //
      if (new_di.length > DO.length) {
        new_di.splice(DO.length, new_di.length - DO.length);
      }

      // 检查现有模块是否匹配 //
      let di_num = 0;
      let i;
      for (i = 0; i < new_di.length; ++i) {
        if (new_di[i].length > DO[i].length) {
          new_di[i].splice(DO.length, new_di.length - DO.length);
        }
        di_num = di_num + new_di[i].length;

        let j;
        for (j = new_di[i].length; j < DO[i].length; ++j) {
          di_num++;
          new_di[i].push({ name: "DO" + di_num, value: DO[i][j] });
        }
      }

      // 添加新的模块 //
      for (i = new_di.length; i < DO.length; ++i) {
        new_di.push([]);
        let j;
        for (j = 0; j < DO[i].length; ++j) {
          new_di[i].push({ value: DO[i][j], name: "DO" + di_num });
          di_num++;
        }
      }

      this.updateOptions({ ...options, DO: new_di });
    }

    let num = 0;
    return (
      <div className={prefixCls}>
        <div>
          {!!options.DO &&
            options.DO.map((data, index) => {
              return (
                <div>
                  <Row className={`${prefixCls}-row`}>
                    {data.map((d, idx) => {
                      return (
                        <div className={`${prefixCls}-di-array`}>
                          <BulbFilled
                            className={
                              d.value
                                ? `${prefixCls}-bulb-success`
                                : `${prefixCls}-bulb-error`
                            }
                          />
                          <Paragraph
                            editable={{
                              onChange: (str) => {
                                const newOpt = { ...options };
                                newOpt.DO[index][idx].name = str;

                                this.updateOptions(newOpt);
                              },
                            }}
                          >
                            {d.name}
                          </Paragraph>
                          <span>{num++}</span>
                        </div>
                      );
                    })}
                  </Row>
                  <hr
                    style={{
                      height: 3,
                    }}
                  />
                </div>
              );
            })}
        </div>
      </div>
    );
  };
}

const mapStateToProps = (state, props) => {
  return {
    DO: getMemoizeDO(state),
  };
};
const ConnectedDODisplay = connect(mapStateToProps)(DODisplay);

ConnectedDODisplay.NAME = "DO显示";
ConnectedDODisplay.WIDTH = 24;
ConnectedDODisplay.HEIGHT = 12;

export default ConnectedDODisplay;
