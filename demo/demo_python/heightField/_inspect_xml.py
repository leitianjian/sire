"""Inspect Sire XML geometry structure."""
import xml.etree.ElementTree as ET
from pathlib import Path

xml_path = Path('d:/code/sire/demo/demo_python/sirePaperDogRL/go2.xml')
if not xml_path.exists():
    xml_path = Path('d:/code/sire/demo/demo_python/resources/go2SRCF.xml')

tree = ET.parse(str(xml_path))
root = tree.getroot()

pe = root.find('PhysicsEngine')
gpo = pe.find('GeometryPoolObject')
print(f'GeometryPoolObject has {len(gpo)} children:')
for i, g in enumerate(gpo):
    tag = g.tag
    a = dict(g.attrib)
    id_ = a.get('id', '?')
    pid = a.get('part_id', '?')
    dyn = a.get('is_dynamic', '?')
    if 'resource_path' in a:
        res = a['resource_path'].split('/')[-1]
        print(f'  [{i}] id={id_} part={pid} dynamic={dyn} MeshGeometry: {res}')
    elif 'file' in a:
        print(f'  [{i}] id={id_} part={pid} dynamic={dyn} HeightField: file={a["file"]}')
    elif 'side' in a:
        print(f'  [{i}] id={id_} part={pid} dynamic={dyn} BoxGeometry: side={a["side"]}')
    elif 'radius' in a:
        print(f'  [{i}] id={id_} part={pid} dynamic={dyn} SphereGeometry: r={a["radius"]}')
    else:
        print(f'  [{i}] id={id_} part={pid} dynamic={dyn} {tag}')
