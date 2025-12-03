import XMLJSONBase from "./XMLJSONBase";
import Device from "./Device";

export class Vendor extends XMLJSONBase {
  constructor(id, name, devices) {
    super();
    this.id = id;
    this.name = name;
    this.devices = devices;
  }

  static FromXML = (node) =>
    new Vendor(
      node.getAttribute("Id"),
      node.getAttribute("Name"),
      [...node.getElementsByTagName("Device")].map(Device.FromXML)
    );

  static FromJSON = ({ id, name, devices }) =>
    new Vendor(id, name, devices && devices.map(Device.FromJSON));

  toXML = () =>
    `<Vendor ${Vendor.ToXMLAttrString("Id", this.id)} ${Vendor.ToXMLAttrString(
      "Name",
      this.name
    )}>${this.devices && this.devices.map((d) => d.toXML()).join("")}</Vendor>`;

  toJSON = () => ({
    name: this.name,
    id: this.id,
    devices: this.devices && this.devices.map((d) => d.toJSON()),
  });
}

export default Vendor;
