import React, { Component } from "react";
import PropTypes from "prop-types";
import Modal from "antd/lib/modal";

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
class DeleteProgramModal extends Component {
  handleSubmit = (e) => {
    e.preventDefault();
    const { deleteProgram } = this.props;
    deleteProgram(this.props.showDeleteProgramModal);
  };

  render() {
    const { toggleDeleteProgramModal, prefixCls, localestate } = this.props;
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <Modal
          title={<FormattedMessage id="删除程序" />}
          visible={!!this.props.showDeleteProgramModal}
          okText={<FormattedMessage id="删除" />}
          cancelText={<FormattedMessage id="取消" />}
          onOk={this.handleSubmit}
          onCancel={() => toggleDeleteProgramModal()}
          wrapClassName={`${prefixCls}-delete-program-modal`}
        >
          <div className={"info"}>
            {<FormattedMessage id="是否确认删除" />}
            {this.props.showDeleteProgramModal !== true
              ? ` "${this.props.showDeleteProgramModal}" `
              : ""}
            !
          </div>
          <div className={`${prefixCls}-status`}>
            {!!this.props.deletingProgram && (
              <div className={"info"}>{<FormattedMessage id="删除中" />}</div>
            )}
            {!!this.props.deletingProgramFailure && (
              <div className={"error"}>
                {<FormattedMessage id="删除错误：" />}
                {this.props.deletingProgramFailure}
              </div>
            )}
            {!!this.props.deletingProgramSuccess && (
              <div className={"success"}>
                {<FormattedMessage id="删除成功" />}
              </div>
            )}
          </div>
        </Modal>
      </IntlProvider>
    );
  }
}

const getBlocklyState = createSelector(getlocalestate, (localestate) => {
  return {
    localestate,
  };
});

function mapStateToProps(state, props) {
  return getBlocklyState(state, props);
}

DeleteProgramModal.propTypes = {
  deleteProgram: PropTypes.func.isRequired,
  toggleDeleteProgramModal: PropTypes.func.isRequired,
  prefixCls: PropTypes.string.isRequired,
};

const ConnectedDeleteProgramModal =
  connect(mapStateToProps)(DeleteProgramModal);

export default ConnectedDeleteProgramModal;
