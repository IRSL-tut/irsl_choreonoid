#!/usr/bin/env python3

# PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-2.0/python
import irsl_choreonoid.hrpsyslog_util as hu

## scripts
from yaml import safe_load as yaml_safe_load
from irsl_choreonoid.robot_util import make_coordinates

import argparse

class Add_Cog_LegsEEF(object):
    def __init__(self, base, robot_name, zipfile=None):
        self.info = hu.GroundTruthInfo(base=base, robot_name=robot_name, zipfile=zipfile)

    def loadRobot(self, fname):
        self.info.loadRobot(fname)

    def main(self, r_offset=None, l_offset=None):
        p_cog = hu.ProcessCOG(self.info)
        r_eef = hu.ProcessEEF(self.info, link_name='R_ANKLE_P', suffix='pose_rleg_eef', offset=r_offset)
        l_eef = hu.ProcessEEF(self.info, link_name='L_ANKLE_P', suffix='pose_lleg_eef', offset=l_offset)
        p_cog.process()
        r_eef.process()
        l_eef.process()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='add_cog_eef.py',
                                     description="add_cog_eef.py --robotname Robot --robotfile 'Robot.body' --basename hrpsys_logname --zipfile hrpsys_logname.zip --r-offset '{pos:[0, 0, -0.1]}' --l-offset '{pos:[0, 0, -0.1]}'",
                                     add_help=True
                                     )
    parser.add_argument('-r', '--robotfile', help='inputfile', required=True)
    parser.add_argument('-b', '--basename', help='inputfile', required=True)
    parser.add_argument('-n', '--robotname', help='inputfile', required=True)
    parser.add_argument('-z', '--zipfile',  help='inputfile')
    parser.add_argument('-L', '--l-offset', help='offset of lleg')
    parser.add_argument('-R', '--r-offset', help='offset of rleg')
    parser.add_argument('-v', '--verbose', action='store_true', help='verbose')

    args = parser.parse_args()

    r_offset=None
    l_offset=None

    if args.r_offset is not None:
        r_offset = make_coordinates(yaml_safe_load(args.r_offset))
    if args.l_offset is not None:
        l_offset = make_coordinates(yaml_safe_load(args.l_offset))

    addcogeef = Add_Cog_LegsEEF(base=args.basename, robot_name=args.robotname, zipfile=args.zipfile)
    addcogeef.loadRobot(args.robotfile)
    addcogeef.main(r_offset=r_offset, l_offset=l_offset)

## python3 add_cog_eef.py --robotname RobotName --robotfile 'RobotName/RobotName.body' --basename logfilename --zipfile logfilename.zip --r-offset='{pos:[0,0,-0.1]}' --l-offset='{pos:[0,0,-0.1]}'
