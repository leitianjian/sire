import React, { Component } from "react";
import PropTypes from "prop-types";

class ResultError extends Component {
  render() {
    let { filter, result } = this.props;
    filter = filter || [];
    return (
      <div {...this.props}>
        {Object.keys(result).map(
          (key) =>
            !!result[key] &&
            filter.indexOf(key) === -1 && (
              <div key={key}>
                <span>{key}</span>:<span>{result[key]}</span>
              </div>
            )
        )}
      </div>
    );
  }
}

ResultError.propTypes = {
  filter: PropTypes.array,
  result: PropTypes.object.isRequired,
};

export default ResultError;
