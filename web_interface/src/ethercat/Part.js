import XMLJSONBase from "./XMLJSONBase";
import GeometryPoolElement from "./GeometryPooolElement";

export class Part extends XMLJSONBase {
  constructor(name, geometryPoolElement) {
    super();
    this.name = name;
    this.geometryPoolElement = geometryPoolElement;
  }

  static FromXML = (node) => {
    return new Part(
      node.getAttribute("name"),
      [...node.getElementsByTagName("GeometryPoolElement")].map(
        GeometryPoolElement.FromXML
      )[0]
    );
  };

  static FromJSON = ({ name, geometryPoolElement }) =>
    new Part(name, GeometryPoolElement.FromJSON(geometryPoolElement));

  toXML = () => {
    return `<Part ${Part.ToXMLAttrString("name", this.name)}>
${this.geometryPoolElement && this.geometryPoolElement.toXML()}
</Part>`;
  };

  toJSON = () => ({
    name: this.name,
    geometryPoolElement:
      this.geometryPoolElement && this.geometryPoolElement.toJSON(),
  });
}

export default Part;
