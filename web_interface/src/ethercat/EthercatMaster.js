import XMLJSONBase from "./XMLJSONBase";
import Vendor from "./Vendor";

export class EthercatMaster extends XMLJSONBase {
  constructor(vendors) {
    super();
    this.vendors = vendors;
  }

  static FromXML = (node) =>
    new EthercatMaster(
      [...node.getElementsByTagName("Vendor")].map(Vendor.FromXML)
    );

  static FromJSON = ({ vendors }) =>
    new EthercatMaster(vendors && vendors.map(Vendor.FromJSON));

  toXML = () =>
    `<ethercat_controller>${
      this.vendors ? this.vendors.map((v) => v.toXML()).join("") : ""
    }</ethercat_controller>`;

  toJSON = () => ({
    vendors: this.vendors && this.vendors.map((v) => v.toJSON()),
  });
}
