import React from "react";
import PropTypes from "prop-types";
import { connect } from "react-redux";
import CellBase from "../CellBase";
import { Statistic } from "antd";
import { getMemoizeForceData } from "../../../state/selectors";

class SixDimensionForce extends CellBase {
  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, sixDimension } = this.props;
    const prefixCls = getPrefixCls("six-dimension-force", customizePrefixCls);
    return (
      <div className={prefixCls}>
        <div>
          <Statistic
            className={
              sixDimension ? `${prefixCls}-number` : `${prefixCls}-number-error`
            }
            title="Fx"
            value={sixDimension ? sixDimension[0] : null}
            precision={2}
          />
        </div>
        <div>
          <Statistic
            className={
              sixDimension ? `${prefixCls}-number` : `${prefixCls}-number-error`
            }
            title="Fy"
            value={sixDimension ? sixDimension[1] : null}
            precision={2}
          />
        </div>
        <div>
          <Statistic
            className={
              sixDimension ? `${prefixCls}-number` : `${prefixCls}-number-error`
            }
            title="Fz"
            value={sixDimension ? sixDimension[2] : null}
            precision={2}
          />
        </div>
        <div>
          <Statistic
            className={
              sixDimension ? `${prefixCls}-number` : `${prefixCls}-number-error`
            }
            title="Mx"
            value={sixDimension ? sixDimension[3] : null}
            precision={2}
          />
        </div>
        <div>
          <Statistic
            className={
              sixDimension ? `${prefixCls}-number` : `${prefixCls}-number-error`
            }
            title="My"
            value={sixDimension ? sixDimension[4] : null}
            precision={2}
          />
        </div>
        <div>
          <Statistic
            className={
              sixDimension ? `${prefixCls}-number` : `${prefixCls}-number-error`
            }
            title="Mz"
            value={sixDimension ? sixDimension[5] : null}
            precision={2}
          />
        </div>
      </div>
    );
  };
}

SixDimensionForce.propTypes = {
  sixDimension: PropTypes.array,
};

const mapStateToProps = (state, props) => {
  return {
    sixDimension: getMemoizeForceData(state),
  };
};
const ConnectedSixDimensionForce = connect(mapStateToProps)(SixDimensionForce);

ConnectedSixDimensionForce.NAME = "六维力显示";
ConnectedSixDimensionForce.WIDTH = 24;
ConnectedSixDimensionForce.HEIGHT = 12;
export default ConnectedSixDimensionForce;
