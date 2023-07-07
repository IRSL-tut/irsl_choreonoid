import numpy as np
from cnoid.IRSLCoords import coordinates
from irsl_choreonoid.draw_coords import DrawCoordsList

## generate wrapper class
cdl = DrawCoordsList(length=0.08, width=5.0)

## add coords
origin = np.array([-0.5, -0.5, -0.5])
for pos in (np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])):
    cds = coordinates(origin + pos)
    cdl.addCoords(cds)

## show coords
cdl.show()

## move coords by setting Origin
cds = coordinates()
cds.translate(np.array([0, 0, 1.0]))
cds.rotate(0.4, np.array([0.1, 0.2, 0.3]))
cdl.setOrigin(cds)
cdl.flush()
