import React, { Component, Fragment } from "react";
import ReactEcharts from "echarts-for-react";
import PropTypes from "prop-types";
import * as echarts from "echarts";

import { createSelector } from "reselect";
import { getlocalestate } from "../../../state/selectors";
import { connect } from "react-redux";

// 进行中英文切换
import zhCN from "../locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "../locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

const contrastColor = "#555";
const axisCommon = function () {
  return {
    axisLine: {
      lineStyle: {
        color: contrastColor,
      },
    },
    axisTick: {
      lineStyle: {
        color: contrastColor,
      },
    },
    axisLabel: {
      textStyle: {
        color: contrastColor,
      },
    },
    splitLine: {
      lineStyle: {
        type: "dashed",
        color: "#aaa",
      },
    },
    splitArea: {
      areaStyle: {
        color: contrastColor,
      },
    },
  };
};

const colorPalette = [
  "#dd6b6699",
  "#759aa099",
  "#e69d8799",
  "#8dc1a999",
  "#ea7e5399",
  "#eedd7899",
  "#73a37399",
  "#73b9bc99",
  "#7289ab99",
  "#91ca8c99",
  "#f49f4299",
];
const theme = {
  color: colorPalette,
  backgroundColor: "#111",
  tooltip: {
    axisPointer: {
      lineStyle: {
        color: contrastColor,
      },
      crossStyle: {
        color: contrastColor,
      },
    },
  },
  legend: {
    textStyle: {
      color: contrastColor,
    },
  },
  textStyle: {
    color: contrastColor,
  },
  title: {
    textStyle: {
      color: contrastColor,
    },
  },
  toolbox: {
    iconStyle: {
      normal: {
        borderColor: contrastColor,
      },
    },
  },
  dataZoom: {
    textStyle: {
      color: contrastColor,
    },
  },
  visualMap: {
    textStyle: {
      color: contrastColor,
    },
  },
  timeline: {
    lineStyle: {
      color: contrastColor,
    },
    itemStyle: {
      normal: {
        color: colorPalette[1],
      },
    },
    label: {
      normal: {
        textStyle: {
          color: contrastColor,
        },
      },
    },
    controlStyle: {
      normal: {
        color: contrastColor,
        borderColor: contrastColor,
      },
    },
  },
  timeAxis: axisCommon(),
  logAxis: axisCommon(),
  valueAxis: axisCommon(),
  categoryAxis: axisCommon(),

  line: {
    symbol: "circle",
  },
  graph: {
    color: colorPalette,
  },
  gauge: {
    title: {
      textStyle: {
        color: contrastColor,
      },
    },
  },
  candlestick: {
    itemStyle: {
      normal: {
        color: "#FD1050",
        color0: "#0CF49B",
        borderColor: "#FD1050",
        borderColor0: "#0CF49B",
      },
    },
  },
};
theme.categoryAxis.splitLine.show = false;

echarts.registerTheme("robot_theme", theme);

class Chart extends Component {
  render() {
    const { dataChannel, results } = this.props;
    if (Object.keys(results).length === 0) {
      return null;
    }
    const getOptions = {
      title: {
        text: <FormattedMessage id="数据来源" /> + dataChannel,
        x: "center",
        y: "bottom",
        textStyle: {
          fontSize: 12,
        },
      },
      legend: {
        data: Object.keys(results),
      },
      xAxis: {
        splitLine: { show: false }, // 去除网格线
        boundaryGap: true,
        min: "dataMin",
        max: "dataMax",
        type: "category",
      },
      yAxis: {
        splitLine: { show: false }, // 去除网格线
        type: "value",
      },
      series: Object.keys(results).map((key) => ({
        name: key,
        data: results[key],
        symbol: "none",
        // sampling: 'average',
        type: "line",
      })),
    };
    const { localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <Fragment>
          <ReactEcharts
            style={{
              width: "100%",
              height: "100%",
            }}
            theme={"robot_theme"}
            option={getOptions}
            notMerge={true}
            lazyUpdate={true}
          />
        </Fragment>
      </IntlProvider>
    );
  }
}

Chart.propTypes = {
  dataChannel: PropTypes.func.isRequired,
  prefixCls: PropTypes.string.isRequired,
  results: PropTypes.string.isRequired,
};

const getState = createSelector(getlocalestate, (localestate) => {
  return {
    localestate,
  };
});

function mapStateToProps(state, props) {
  return getState(state, props);
}

export default connect(mapStateToProps)(Chart);
