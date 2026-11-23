
import os
import sire

sim = sire.Simulator()
sire.fromXmlFile(sim, os.path.abspath('sire_ball_free_fall.xml'))
model = sim.model()

# Modifying xml before init?
# Wait! Can we change the position through sire API after fromXmlFile and before sim.init()?
# If not, let's modify the XML directly!

