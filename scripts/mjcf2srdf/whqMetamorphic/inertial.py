import mujoco
spec = mujoco.MjSpec.from_file("D:/code/sire/scripts/mjcf2srdf/whqMetamorphic/metamophicMJCF.xml")
mjmodel = spec.compile()
for body in spec.bodies:
    body.explicitinertial = True
# spec.compiler.inertiafromgeom = 2
configString = spec.to_xml()
with open("D:/code/sire/scripts/mjcf2srdf/whqMetamorphic/metamophicMJCF_inertial.xml", "w+") as text_file:
    text_file.write(configString)