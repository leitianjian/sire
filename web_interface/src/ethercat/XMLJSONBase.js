export class XMLJSONBase {
  static FromXML = (node) => {
    throw "FromXML not implemented";
  };

  static FromJSON = (node) => {
    throw "FromJSON not implemented";
  };

  static GetNumber = (n, d = 0) => {
    try {
      return JSON.parse(n);
    } catch (e) {
      return d;
    }
  };

  static GetBool = (n, d = false) => {
    try {
      return JSON.parse(n);
    } catch (e) {
      return d;
    }
  };

  static GetDegreeNumber = (n, d = 0) => {
    try {
      return (JSON.parse(n) / Math.PI) * 180.0;
    } catch (e) {
      return d;
    }
  };

  static ToRadian = (n) => {
    return (n / 180.0) * Math.PI;
  };

  static ToXMLAttrString = (attr, value) => {
    if (attr === "is_virtual") {
      // console.log(attr, value)
    }
    if (value === false) {
      return `${attr}="${value}"`;
    } else if (!value && value !== 0) {
      return "";
    } else {
      return `${attr}="${value}"`;
    }
  };

  toXML = () => {
    throw "toXML not implemented";
  };

  toJSON = () => {
    throw "toJSON not implemented";
  };
}

export default XMLJSONBase;
