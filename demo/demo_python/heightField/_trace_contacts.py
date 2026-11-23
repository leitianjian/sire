"""Diagnostic: trace when contacts appear in Sire recorder."""
import sire
sim = sire.Simulator()
sire.fromXmlFile(sim, 'd:/code/sire/demo/demo_python/sirePaperDogRL/go2.xml')
sim.init()
sl = sim.simulationLoop()

print("=== Trace contacts through handleContact ===")
for step_i in range(5):
    sl.handleContact()
    cr = sl.lastContactPairResults()
    print(f'  handleContact #{step_i}: {len(cr)} contacts')
    if cr:
        for x in cr[:2]:
            print(f'    partA={x[0]} partB={x[1]} fz={x[4]:.3f}')

print()
print("=== After stepping 200 times ===")
for _ in range(200):
    sl.step()
cr = sl.lastContactPairResults()
print(f'{len(cr)} contacts')
for x in cr[:4]:
    print(f'  partA={x[0]} partB={x[1]} fz={x[4]:.3f}')
