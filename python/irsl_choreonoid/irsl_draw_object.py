## do not use cnoid.Base
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
    def __init__(self, target, init_coords=None, update_callback=None):
        super().__init__()
        self.__target = target
        ##if hasattr(target, 'coords'):
        ##    target.coords = self
        ##else:
        ##    setattr(target, 'coords', self)

        self.__update_callback = update_callback

        if init_coords is not None:
            self.newcoords(init_coords)

    def updateTarget(self):
        """

        Args:

        Returns:

        """
        self.__target.T = self.cnoidPosition
        if callable(self.__update_callback):
            self.__update_callback()

    def revert(self): ##
        self.cnoidPosition =  self.__target.T

    def setUpdateCallback(self, func):
        """

        Args:

        Returns:

        """
        if callable(func):
            self.__update_callback = func

    def newcoords(self, cds):
        """

        Args:

        Returns:

        """
        super().newcoords(cds)
        self.updateTarget()
        return self

    def translate(self, pos, wrt = None):
        """

        Args:

        Returns:

        """
        if wrt is None:
            super().translate(pos)
        else:
            super().translate(pos, wrt)
        self.updateTarget()
        return self

    def rotate(self, rot, wrt = None):
        """

        Args:

        Returns:

        """
        if wrt is None:
            super().rotate(rot)
        else:
            super().rotate(rot, wrt)
        self.updateTarget()
        return self

    def transform(self, trs, wrt = None):
        """

        Args:

        Returns:

        """
        if wrt is None:
            super().transform(trs)
        else:
            super().transform(trs, wrt)
        self.updateTarget()
        return self

    def __repr__(self):
        if self is self.target:
            return 'Wrap {} : '.format(type(self)) + super().__repr__()
        else:
            return 'Wrap: ' + super().__repr__() + ' | ' + self.target.__repr__()

    @property
    def target(self):
        """target
        """
        return self.__target
