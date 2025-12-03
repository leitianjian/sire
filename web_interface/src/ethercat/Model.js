import XMLJSONBase from "./XMLJSONBase";
import PartPoolElement from "./PartPoolElement";

export class Model extends XMLJSONBase {
  constructor(name, partPoolElement) {
    super();
    this.name = name;
    this.partPoolElement = partPoolElement;
  }

  static FromXML = (node) => {
    // console.log(...node.getElementsByTagName("PartPoolElement"))
    return new Model(
      node.getAttribute("name"),
      [...node.getElementsByTagName("PartPoolElement")].map(
        PartPoolElement.FromXML
      )[0]
    );
  };

  static FromJSON = ({ name, partPoolElement }) =>
    new Model(name, PartPoolElement.FromJSON(partPoolElement));

  toXML = () => {
    return `<Model ${Model.ToXMLAttrString(
      "name",
      this.name
    )}>${this.partPoolElement.toXML()}</Model>`;
  };

  toJSON = () => ({
    name: this.name,
    partPoolElement: this.partPoolElement && this.partPoolElement.toJSON(),
  });
}

export default Model;
