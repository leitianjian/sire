import React, { Component } from "react";
import PropTypes from "prop-types";

import { ConfigConsumer } from "antd/lib/config-provider";

// 英文界面
import { connect } from "react-redux";
import { getlocalestate } from "../state/selectors";
import { createSelector } from "reselect";
import zhCN from "../containers/locale/zhCN";
import { IntlProvider, FormattedMessage } from "react-intl";
import enUS from "../containers/locale/enUS";
const messages = {};
messages.chinese = zhCN;
messages.english = enUS;

class DashboardEmpty extends Component {
  renderDashboard = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;
    const prefixCls = getPrefixCls("dashboard-empty", customizePrefixCls);
    return (
      <div className={`${prefixCls}`}>
        {<FormattedMessage id="当前面板没有任何功能单元" />}
      </div>
    );
  };

  render() {
    const { localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <ConfigConsumer>{this.renderDashboard}</ConfigConsumer>
      </IntlProvider>
    );
  }
}

export const getEmptyCellState = createSelector(
  getlocalestate,
  (localestate) => {
    return {
      localestate,
    };
  }
);

function mapStateToProps(state) {
  return getEmptyCellState(state);
}

DashboardEmpty.propTypes = {
  dashboard: PropTypes.object.isRequired,
  onAddCell: PropTypes.func.isRequired,
};

export default connect(mapStateToProps)(DashboardEmpty);
