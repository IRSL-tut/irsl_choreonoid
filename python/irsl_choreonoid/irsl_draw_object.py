from cnoid.Util import SgPosTransform
from cnoid.IRSLCoords import coordinates
# from cnoid.DrawInterface import GeneralDrawInterface as GDI

### newcoords
### translate
### rotate
### transform
#class HOGE(SgPosTransform, coordinates):
#    def __init__(self, *args, **kwargs):
#        coordinates.__init__(self, *args, **kwargs)
#        SgPosTransform.__init__(self)
#    def newcoords(self, cds):
#        super().newcoords(cds)
#        self.T = self.toPosition()

# from cnoid.Util import SgPosTransform
# from cnoid.IRSLCoords import coordinates
# import numpy as np
# sgp = SgPosTransform()
# cds = coordinates()
# cds1 = coordinates()
# cds1.translate(np.array([0.1, 0.2, 0.3]))
# sgp.T
# cds1.cnoidPosition
# sgp.T = cds1.cnoidPosition
# sgp.T

# setattr('coords')
class coordsWrapper(coordinates):
    """coordsWrapper(class)
    """
    def __init__(self, child, init_coords=None):
        self.__child = child
        self.newcoords(init_coords)

    def __updateChild(self):
        self.__child.T = self.T

    def revert(self): ##
        self.T =  self.__child.T

    def newcoords(self, cds):
        """

        Args:

        Returns:

        """
        super().newcoords(cds)
        self.__updateChild()
        return self

    def translate(self, pos, wrt = None):
        """

        Args:

        Returns:

        """
        if wrt is None:
            self.translate(pos)
        else:
            self.translate(pos, wrt)
        self.__updateChild()
        return self

    def rotate(self, rot, wrt = None):
        """

        Args:

        Returns:

        """
        if wrt is None:
            self.rotate(rot)
        else:
            self.rotate(rot, wrt)
        self.__updateChild()
        return self

    def transform(self, trs, wrt = None):
        """

        Args:

        Returns:

        """
        if wrt is None:
            self.transform(trs)
        else:
            self.transform(trs, wrt)
        self.__updateChild()
        return self
