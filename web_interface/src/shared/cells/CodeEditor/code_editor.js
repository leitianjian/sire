import React from "react";
import { connect } from "react-redux";
import { IntlProvider } from "react-intl";
import CellBase from "../CellBase";

import { withSize } from "react-sizeme";

import AceEditor from "react-ace";
import "ace-builds/src-noconflict/mode-java";
import "ace-builds/src-noconflict/theme-github";
import "ace-builds/src-noconflict/mode-html";
import "ace-builds/src-noconflict/theme-monokai";
import "ace-builds/src-noconflict/snippets/html";

window.ace.require = window.ace.acequire;

class CodeEditor extends CellBase {
  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;
    const prefixCls = getPrefixCls("code-editor", customizePrefixCls);

    return (
      <IntlProvider>
        <div className={`${prefixCls}`}>
          <AceEditor
            mode="java"
            theme="github"
            onChange={(e) => {
              console.log("change");
            }}
            name="UNIQUE_ID_OF_DIV"
            editorProps={{ $blockScrolling: true }}
          />
        </div>
      </IntlProvider>
    );
  };
}

function mapStateToProps(state, props) {
  return {};
}

const mapDispatchToProps = (dispatch) => {
  return {};
};

const ConnectedCodeEditor = connect(
  mapStateToProps,
  mapDispatchToProps
)(withSize({ monitorHeight: true, refreshRate: 30 })(CodeEditor));

ConnectedCodeEditor.NAME = "文本编辑";
ConnectedCodeEditor.WIDTH = 24;
ConnectedCodeEditor.HEIGHT = 12;

export default ConnectedCodeEditor;
