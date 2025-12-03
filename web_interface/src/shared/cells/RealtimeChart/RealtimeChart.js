import React, { Fragment } from "react";
import CellBase from "../CellBase";
import { connect } from "react-redux";
import Button from "antd/lib/button";
import isEqual from "lodash/isEqual";
import Icon from "antd/lib/icon";
// import {deleteResultByChannel} from "../../../state/actions"
import {
  // makeGetResultsByCellChannel,
  getlocalestate,
} from "../../../state/selectors";
import Config from "./Config";
import Chart from "./Chart";

import { createSelector } from "reselect";

// 进行中英文切换
import zhCN from "../locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "../locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

class RealtimeChart extends CellBase {
  interval = null;

  state = { playing: true };

  resetInterval = () => {
    clearInterval(this.interval);
    const { cell, sendCmd } = this.props;
    if (
      cell.options &&
      !cell.options.showConfig &&
      cell.options.channel === "command" &&
      cell.options.command &&
      cell.options.interval &&
      !cell.options.paused
    ) {
      const command = cell.options.command;
      const channel = cell.i;
      this.interval = setInterval(() => {
        sendCmd(command, channel);
      }, cell.options.interval);
    }
  };

  componentWillUnmount() {
    clearInterval(this.interval);
  }

  componentDidMount() {
    this.resetInterval();
    this.start = new Date();
  }

  shouldComponentUpdate(nextProps, nextState, nextContext) {
    const now = new Date();
    if (now - this.start > 1000) {
      this.start = now;
      return true;
    }
    return false;
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const { cell: oldCell } = prevProps;
    const { cell: newCell } = this.props;
    if (!isEqual(oldCell, newCell)) {
      this.resetInterval();
    }
    // if(newCell.options.showConfig !== oldCell.options.showConfig && newCell.options.showConfig){
    // this.props.deleteResultByChannel(newCell.i)
    // }
  }

  handleChange = (value) => {
    const options = this.getOptions();
    this.updateOptions({ ...options, channel: value });
  };

  renderContent = ({ getPrefixCls }) => {
    const {
      prefixCls: customizePrefixCls,
      cell,
      // results,
      localestate,
    } = this.props;
    // console.log("绘图组件中的localestate:",localestate)
    const prefixCls = getPrefixCls("realtime-chart", customizePrefixCls);
    const hasDataSources =
      cell.options &&
      cell.options.channel &&
      cell.options.sources &&
      cell.options.sources.length > 0;
    const dataChannel = `${
      cell.options.channel === "get" ? "get" : cell.options.command
    }`;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <div
          className={`${prefixCls} ${
            !hasDataSources && prefixCls + "-no-data"
          }`}
        >
          {!hasDataSources && (
            <div>
              <span>
                <FormattedMessage id="未配置数据源 或 未关联数据项" />
              </span>
              <div style={{ marginTop: 20 }}>
                <Button
                  type={"primary"}
                  onClick={() => {
                    this.updateOptions({ showConfig: true });
                  }}
                >
                  <FormattedMessage id="添加配置数据" />
                </Button>
              </div>
            </div>
          )}
          {hasDataSources && (
            <Fragment>
              <Chart
                dataChannel={dataChannel}
                prefixCls={prefixCls}
                // results={results}
              />

              {cell.options.paused && (
                <Icon
                  type="play-circle"
                  className={`${prefixCls}-play ${prefixCls}-play-start`}
                  onClick={() => {
                    this.updateOptions({
                      paused: false,
                    });
                  }}
                />
              )}
              {!cell.options.paused && (
                <Icon
                  type="pause-circle"
                  className={`${prefixCls}-play`}
                  onClick={() => {
                    this.updateOptions({
                      paused: true,
                    });
                  }}
                />
              )}
            </Fragment>
          )}
        </div>
      </IntlProvider>
    );
  };
}

RealtimeChart.propTypes = {
  // results: PropTypes.object.isRequired
};

const getState = createSelector(getlocalestate, (localestate) => {
  return {
    localestate,
  };
});

const makeMapStateToProps = () => {
  // const getResultsByCellChannel = makeGetResultsByCellChannel(1000)
  const mapStateToProps = (state, props) => {
    return {
      localestate: getState(state, props).localestate,
      // results: getResultsByCellChannel(state, props),
    };
  };
  return mapStateToProps;
};

const ConnectedRealtimeChart = connect(
  makeMapStateToProps
  // {deleteResultByChannel}
)(RealtimeChart);

ConnectedRealtimeChart.NAME = "实时绘图";
ConnectedRealtimeChart.CONFIG = Config;

export default ConnectedRealtimeChart;
