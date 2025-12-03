import XMLJSONBase from "./XMLJSONBase";
import FileGeometry from "./FileGeometry";

export class GeometryPoolElement extends XMLJSONBase {
  constructor(name, fileGeometry) {
    super();
    this.name = name;
    this.fileGeometry = fileGeometry;
  }

  static FromXML = (node) => {
    return new GeometryPoolElement(
      node.getAttribute("name"),
      [...node.getElementsByTagName("FileGeometry")].map(
        FileGeometry.FromXML
      )[0]
    );
  };

  static FromJSON = ({ name, fileGeometry }) =>
    new GeometryPoolElement(name, fileGeometry.FromJSON(fileGeometry));

  toXML = () => {
    return `<GeometryPoolElement ${GeometryPoolElement.ToXMLAttrString(
      "name",
      this.name
    )} >${this.fileGeometry.toXML()}</GeometryPoolElement>`;
  };

  toJSON = () => ({
    name: this.name,
    fileGeometry: this.fileGeometry && this.fileGeometry.toJSON(),
  });
}

export default GeometryPoolElement;
