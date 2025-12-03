import React from "react";
import { connect } from "react-redux";
import CellBase from "../CellBase";
import { Layout } from "antd";
import throttle from "lodash/throttle";
import Row from "antd/lib/row";
import {
  dispatchLogName,
  diapatchLogFileContent,
} from "../../../state/actions";
import { getLogNameList, getLogFileContent } from "../../../state/selectors";

const { Header, Content } = Layout;
class LogDisplay extends CellBase {
  //  nameString为文件名
  //  logContentPageNum为当前的文件内容在第几页
  // logNameList为当前文件名列表
  // selectedProgram为当前选中的文件的文件名，用于css样式显示
  state = {
    nameString: "",
    logContentPageNum: 0,
    logContent: "",
    logNameListNum: 0,
    logNameList: "",
    stateCode: 200,
    selectedProgram: "",
  };

  // 执行dispatchLogName来获取文件名列表
  componentDidMount = () => {
    console.log("执行LogDisplay");
    this.props.dispatchLogName(0);
    // 设置一个150ms的延时 防止多次触发
    this.throttledOnScrollListener = throttle(this.loadData, 150);
    document
      .querySelector(".logDisplay-Content")
      .addEventListener("scroll", this.throttledOnScrollListener);
    this.throttledOnScrollListener2 = throttle(this.loadNameList, 150);
    document
      .querySelector("#logDisplay-menu")
      .addEventListener("scroll", this.throttledOnScrollListener2);
  };

  // 判断当前文件名是否一致
  shouldComponentUpdate = (nextProps, nextState) => {
    if (
      this.state.nameString === nextState.nameString &&
      nextProps.logContent !== this.props.logContent
    ) {
      // console.log("更新logContent")
      const data = this.state.logContent + nextProps.logContent;
      this.setState({
        logContent: data,
      });
    }
    // 对文件名列表进行修改
    if (nextProps.logNameList !== this.props.logNameList) {
      // console.log("更新logNameList")
      const nameData = this.state.logNameList + nextProps.logNameList;
      this.setState({
        logNameList: nameData,
      });
    }
    return true;
  };

  // 判断是否下拉到底部
  loadNameList = () => {
    const isTrue =
      document.querySelector("#logDisplay-menu").scrollTop +
        document.querySelector("#logDisplay-menu").clientHeight -
        document.querySelector("#logDisplay-menu").scrollHeight ===
      0;
    const isToBottom = isTrue;
    // console.log("更新loadNameList")
    if (isToBottom) {
      console.log("到达底部2");
      this.props.dispatchLogName(this.state.logNameListNum + 1);
      this.setState({
        logNameListNum: this.state.logNameListNum + 1,
      });
    }
  };

  // 加载CONTENT数据
  loadData = () => {
    const isTrue =
      document.querySelector(".logDisplay-Content").scrollTop +
        document.querySelector(".logDisplay-Content").clientHeight -
        document.querySelector(".logDisplay-Content").scrollHeight ===
      0;
    const isToBottom = isTrue;
    // console.log("isToBottom",isTrue)
    // console.log("isToBottom",isToBottom)
    if (isToBottom) {
      // console.log("到达底部")
      this.props.diapatchLogFileContent(
        this.state.nameString,
        this.state.logContentPageNum + 1
      );
      this.setState({
        logContentPageNum: this.state.logContentPageNum + 1,
      });
    }
  };

  // 每次点击一个新文件名的时候请求Content的数据
  displayContent = (value) => {
    // console.log("e.item",value)
    this.setState({
      nameString: value,
      logContentPageNum: 0,
      logContent: "",
      selectedProgram: value,
    });
    this.props.diapatchLogFileContent(value, 1);
  };

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls, logNameList } = this.props;

    const prefixCls = getPrefixCls("log-display", customizePrefixCls);
    console.log("logNameList", logNameList);
    return (
      <div style={{ width: "100%", height: "100%" }} className={prefixCls}>
        <Layout className={`${prefixCls}-layout`}>
          <div className={`${prefixCls}-sider`}>
            <Row
              id="logDisplay-menu"
              className={`${prefixCls}-menu ${prefixCls}-menu-list`}
            >
              {!!logNameList &&
                logNameList.map((value, index) => {
                  return (
                    <div
                      key={value}
                      className={`${
                        this.state.selectedProgram === value
                          ? `${prefixCls}-selected-program`
                          : ""
                      } ${prefixCls}-menu-list-item`}
                      onClick={() => this.displayContent(value)}
                    >
                      {value}
                    </div>
                  );
                })}
            </Row>
          </div>
          <Layout className={`${prefixCls}-layout-right`} style={{}}>
            <Header>显示log中的数据</Header>
            <Content
              className={"logDisplay-Content"}
              ref={(ref) => (this.content = ref)}
              style={{
                height: "100%",
                overflow: "auto",
              }}
            ></Content>
          </Layout>
        </Layout>
      </div>
    );
  };
}

const mapStateToProps = (state, props) => {
  return {
    logNameList: getLogNameList(state),
    logContent: getLogFileContent(state),
  };
};
const ConnectedLogDisplay = connect(mapStateToProps, {
  dispatchLogName,
  diapatchLogFileContent,
})(LogDisplay);

ConnectedLogDisplay.NAME = "日志回放";
ConnectedLogDisplay.WIDTH = 24;
ConnectedLogDisplay.HEIGHT = 12;

export default ConnectedLogDisplay;
