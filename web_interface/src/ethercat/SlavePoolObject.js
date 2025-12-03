import XMLJSONBase from "./XMLJSONBase";
import EthercatSlave from "./EthercatSlave";
import EthercatMotor from "./EthercatMotor";

export class SlavePoolObject extends XMLJSONBase {
  constructor(name, slaves) {
    super();
    this.name = name;
    this.slaves = slaves;
  }

  static FromXML = (node) => {
    const slaves = [...node.children]
      .filter((node) => node.nodeType === 1)
      .map((ele) => {
        if (ele.tagName === "EthercatMotor") return EthercatMotor.FromXML(ele);
        else return EthercatSlave.FromXML(ele);
      });

    return new SlavePoolObject(node.getAttribute("name"), slaves);
  };

  static FromJSON = ({ name, slaves }) => {
    const newslaves = slaves.map((ele) => {
      if (ele.isMotion) return EthercatMotor.FromJSON(ele);
      else return EthercatSlave.FromJSON(ele);
    });
    return new SlavePoolObject(name, newslaves);
  };

  toXML = () =>
    `<SlavePoolObject ${SlavePoolObject.ToXMLAttrString("name", this.name)}>${
      this.slaves ? this.slaves.map((s) => s.toXML()).join("") : ""
    }</SlavePoolObject>`;

  toJSON = () => ({
    name: this.name,
    slaves: this.slaves && this.slaves.map((s) => s.toJSON()),
  });
}

export default SlavePoolObject;
