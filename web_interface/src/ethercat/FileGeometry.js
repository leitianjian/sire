import XMLJSONBase from "./XMLJSONBase";

export class FileGeometry extends XMLJSONBase {
  constructor(name, pe, graphicFilePath) {
    super();
    this.name = name;
    this.pe = pe;
    this.graphicFilePath = graphicFilePath;
  }

  static FromXML = (node) =>
    new FileGeometry(
      node.getAttribute("name"),
      node.getAttribute("pe"),
      node.getAttribute("graphic_file_path")
    );

  static FromJSON = ({ name, pe, graphicFilePath }) =>
    new FileGeometry(name, pe, graphicFilePath);

  toXML = () => {
    return `<FileGeometry ${FileGeometry.ToXMLAttrString(
      "name",
      this.name
    )} ${FileGeometry.ToXMLAttrString(
      "pe",
      this.pe
    )} ${FileGeometry.ToXMLAttrString(
      "graphic_file_path",
      this.graphicFilePath
    )}></FileGeometry>`;
  };

  toJSON = () => ({
    name: this.name,
    pe: this.pe,
    graphicFilePath: this.graphicFilePath,
  });
}

export default FileGeometry;
