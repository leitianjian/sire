import XMLJSONBase from "./XMLJSONBase";

export class Device extends XMLJSONBase {
  constructor(name, productCode, revisionNo) {
    super();
    this.name = name;
    this.productCode = productCode;
    this.revisionNo = revisionNo;
  }

  static FromXML = (node) =>
    new Device(
      node.getAttribute("Name"),
      node.getAttribute("ProductCode"),
      node.getAttribute("RevisionNo")
    );

  static FromJSON = ({ name, productCode, revisionNo }) =>
    new Device(name, productCode, revisionNo);

  toXML = () =>
    `<Device ${Device.ToXMLAttrString(
      "Name",
      this.name
    )} ${Device.ToXMLAttrString(
      "ProductCode",
      this.productCode
    )} ${Device.ToXMLAttrString("RevisionNo", this.revisionNo)} />`;

  toJSON = () => ({
    name: this.name,
    productCode: this.productCode,
    revisionNo: this.revisionNo,
  });
}

export default Device;
