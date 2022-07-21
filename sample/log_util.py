import numpy as np
import csv
import os
##
#import irsl_choreonoid.robot_util as ru
from cnoid.IRSLUtil import coordinates

# base = '/home/irsl/src/docker_private/hrpsys_log/jaxon_walk_2_0_0_JAXON_RED_20220721100914'
# sufx = ['.es_q', '.el_q']
# names = [ base + s for s in sufx ]
# filterFiles(names, 'merged', base, filter_function = simple_merge_without_time)

# import irsl_choreonoid.robot_util as ru
# exec(open('/home/irsl/sandbox/choreonoid_ws/src/irsl_choreonoid/sample/log_util.py').read())
# 
# base = 'gopos_1_0_0_PL0_20220716120647'
# sufx = ['PL0(Robot)0_WAIST', 'RobotHardware_choreonoid0_q']
# names = [ base + '.' + s for s in sufx ]
# 
# r = ru.loadRobot('trans_p_model/PL0Pmain_for_choreonoid.wrl')
# 
# f_waist = create_writeLinkPositionFunc(r, 0)
# f_cog = create_writeCOGPositionFunc(r, 0)
# 
# filterFiles(names, 'pos_rpy_WAIST', base, filter_function = f_waist)
# filterFiles(names, 'pos_COG', base, filter_function = f_cog)

## TODO
# - open zipfile
# - check existing the same file
# - create new file(?)
# - add the file to zipfile

def simple_merge(lst):
    ret = []
    for l in lst:
        ret.extend(l)
    return ret

def simple_merge_without_time(lst):
    ret = []
    ret.extend(lst[0])
    for l in lst[1:]:
        ret.extend(l[1:])
    return ret

def create_writeLinkPositionFunc(_robot, _link_idx_or_name):
    _link_idx = _link_idx_or_name
    if type(_link_idx_or_name) == str:
        _link_idx = _robot.jointList().index(_robot.joint(_link_idx_or_name))
    ## def closure_func__(coordslst, angles): ## coordslst[pos(3), rot(9)] joint_angles(N)
    def closure_func__(lst): ## coordslst[pos(3), rot(9)] joint_angles(N)
        tm = lst[0][0:1]
        coordslst = lst[0][1:]
        angles = lst[1][1:] ## skip tm
        ##
        trs_ = np.array(coordslst[0:3]) # lst[0:3]
        rot_ = np.array(coordslst[3:12]) # lst[3:12]
        rot_ = rot_.reshape(3,3)
        #
        cds = coordinates(trs_, rot_)
        cds.rotNormalize()

        for j, ang in zip(_robot.jointList(), angles):
            j.q = ang

        _robot.rootLink.setCoords(cds)
        _robot.calcForwardKinematics()

        lk = _robot.link(_link_idx)
        cds = lk.getCoords()

        return tm + cds.pos.tolist() + cds.getRPY().tolist()
    return closure_func__

def create_writeCOGPositionFunc(_robot, _link_idx_or_name):
    ## def closure_func__(coordslst, angles): ## coordslst[pos(3), rot(9)] joint_angles(N)
    def closure_func__(lst): ## coordslst[pos(3), rot(9)] joint_angles(N)
        tm = lst[0][0:1]
        coordslst = lst[0][1:]
        angles = lst[1][1:] ## skip tm
        ##
        trs_ = np.array(coordslst[0:3]) # lst[0:3]
        rot_ = np.array(coordslst[3:12]) # lst[3:12]
        rot_ = rot_.reshape(3,3)
        #
        cds = coordinates(trs_, rot_)
        cds.rotNormalize()

        for j, ang in zip(_robot.jointList(), angles):
            j.q = ang

        _robot.rootLink.setCoords(cds)
        _robot.calcForwardKinematics()
        _robot.calcCenterOfMass()
        cog = _robot.centerOfMass

        return tm + cog.tolist()

    return closure_func__

#def create_writeCPPositionFunc(_robot, _link_idx_or_name):
#    pass
#def create_writeZMPPositionFunc(_robot, _link_idx_or_name):
#    pass
#def create_writeOmegaFunc(_robot, _link_idx_or_name):
#    pass

def filterFiles(org_fnames, new_suffix, new_prefix = None, filter_function = None, start = 0, length = 0, args = None):
    endidx = start + length
    if not new_prefix is None:
        new_fname = new_prefix + '.' + new_suffix
    else:
        pre_, fname_ = os.path.split(org_fname)
        fpref_ = ''.join(fname_.split('.')[:-1])
        new_fname = pre_ + '/' + fpref_ + new_suffix
    print('new_fname: {}'.format(new_fname))

    ## open files
    flst = []
    for nm in org_fnames:
        try:
            f = open(nm, 'r')
        except Exception as e:
            print('error occured while opening {}'.format(nm))
            print('{} may not exist.'.format(os.path.basename(nm)))
            for l in flst:
                l.close()
            return None
        flst.append(f)

    try:
        with open(new_fname, 'w') as fnew:
            readers = [ csv.reader(f, delimiter=' ').__iter__() for f in flst ]
            writer = csv.writer(fnew, delimiter=' ')
            cntr = 0
            try:
                for rows in zip(*readers):
                    if cntr >= start:
                        # dls = [ filter(lambda x: x != '', row) for row in rows ] ## python2
                        dls = [ [ float(x) for x in filter(lambda x: x != '', row) ] for row in rows ] ## python3
                        filtered_dl = filter_function(dls)
                        writer.writerow(filtered_dl)
                    cntr = cntr + 1
                    if cntr == endidx:
                        break
            except Exception as e:
                print('error occured while reading data : {}'.format(e))
    except Exception as e:
        print('error occured while opening {}'.format(new_fname))
        print('{} may not exist.'.format(os.path.basename(new_fname)))
        for l in flst:
            l.close()
        return None

    for l in flst:
        l.close()
