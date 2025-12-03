import React from "react";
import PropTypes from "prop-types";
import { connect } from "react-redux";
import CellBase from "../CellBase";
import Row from "antd/lib/row";
import { BulbFilled } from "@ant-design/icons";
import { getMemoizeDI } from "../../../state/selectors";

import { Typography } from "antd";
const { Paragraph } = Typography;

class DIDisplay extends CellBase {
  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, di } = this.props;
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
    if (!di) {
    } else if (
      !!options.di &&
      options.di.length === di.length &&
      is_di_same_length(options.di, di)
    ) {
      const newOpt = { ...options };
      let i, j;
      let has_changed = false;
      for (i = 0; i < di.length; ++i) {
        for (j = 0; j < di[i].length; ++j) {
          if (newOpt.di[i][j].value !== di[i][j]) {
            newOpt.di[i][j].value = di[i][j];
            has_changed = true;
          }
        }
      }
      if (has_changed) this.updateOptions(newOpt);
    } else {
      const new_di = options.di ? options.di : [];

      // 删除多余模块 //
      if (new_di.length > di.length) {
        new_di.splice(di.length, new_di.length - di.length);
      }

      // 检查现有模块是否匹配 //
      let di_num = 0;
      let i;
      for (i = 0; i < new_di.length; ++i) {
        if (new_di[i].length > di[i].length) {
          new_di[i].splice(di.length, new_di.length - di.length);
        }
        di_num = di_num + new_di[i].length;

        let j;
        for (j = new_di[i].length; j < di[i].length; ++j) {
          di_num++;
          new_di[i].push({ name: "di" + di_num, value: di[i][j] });
        }
      }

      // 添加新的模块 //
      for (i = new_di.length; i < di.length; ++i) {
        new_di.push([]);
        let j;
        for (j = 0; j < di[i].length; ++j) {
          new_di[i].push({ value: di[i][j], name: "di" + di_num });
          di_num++;
        }
      }

      this.updateOptions({ ...options, di: new_di });
    }
    let num = 0;
    return (
      <div className={prefixCls}>
        <div>
          {!!options.di &&
            options.di.map((data, index) => {
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
                                newOpt.di[index][idx].name = str;

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

DIDisplay.propTypes = {
  di: PropTypes.array,
};

const mapStateToProps = (state, props) => {
  return {
    di: getMemoizeDI(state),
  };
};
const ConnectedDIDisplay = connect(mapStateToProps)(DIDisplay);

ConnectedDIDisplay.NAME = "DI显示";
ConnectedDIDisplay.WIDTH = 24;
ConnectedDIDisplay.HEIGHT = 12;

export default ConnectedDIDisplay;
