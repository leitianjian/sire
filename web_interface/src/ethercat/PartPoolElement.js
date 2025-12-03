import XMLJSONBase from "./XMLJSONBase";
import Part from "./Part";

export class PartPoolElement extends XMLJSONBase {
  constructor(name, parts) {
    super();
    this.name = name;
    this.parts = parts;
  }

  static FromXML = (node) => {
    return new PartPoolElement(
      node.getAttribute("name"),
      [...node.getElementsByTagName("Part")].map(Part.FromXML)
    );
  };

  static FromJSON = ({ name, parts }) =>
    new PartPoolElement(name, parts && parts.map(Part.FromJSON));

  toXML = () => {
    return `<PartPoolElement ${PartPoolElement.ToXMLAttrString(
      "name",
      this.name
    )}>${
      this.parts ? this.parts.map((e) => e.toXML()).join("") : ""
    }</PartPoolElement>`;
  };

  toJSON = () => ({
    name: this.name,
    parts: this.parts && this.parts.map((e) => e.toJSON()),
  });
}

export default PartPoolElement;
