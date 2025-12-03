import XMLJSONBase from "./XMLJSONBase";
import SlavePoolObject from "./SlavePoolObject";

export class EthercatController extends XMLJSONBase {
  constructor(name, slavePoolObjects) {
    super();
    this.name = name;
    this.slavePoolObjects = slavePoolObjects;
  }

  static FromXML = (node) =>
    new EthercatController(
      node.getAttribute("name"),
      [...node.getElementsByTagName("SlavePoolObject")]
        .filter((node) => node.nodeType === 1)
        .map(SlavePoolObject.FromXML)
    );

  static FromJSON = ({ name, slavePoolObjects }) =>
    new EthercatController(
      name,
      slavePoolObjects && slavePoolObjects.map(SlavePoolObject.FromJSON)
    );

  toXML = () =>
    `<EthercatController sample_period_ns="1000000" ${EthercatController.ToXMLAttrString(
      "name",
      this.name
    )}>${
      this.slavePoolObjects
        ? this.slavePoolObjects.map((spo) => spo.toXML()).join("")
        : ""
    }</EthercatController>`;

  toJSON = () => ({
    name: this.name,
    slavePoolObjects:
      this.slavePoolObjects && this.slavePoolObjects.map((spo) => spo.toJSON()),
  });
}

export default EthercatController;
