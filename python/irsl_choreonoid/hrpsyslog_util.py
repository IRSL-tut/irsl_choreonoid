import numpy as np
import csv
import os
import io
import zipfile
zipfile.ZipExtFile.next_original = zipfile.ZipExtFile.__next__
##
#import irsl_choreonoid.robot_util as ru
from cnoid.IRSLUtil import coordinates

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
        tm = lst[1][0:1] ## use tm in angles
        coordslst = lst[0][1:] ## skip tm
        angles    = lst[1][1:] ## skip tm
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
        tm = lst[1][0:1] ## use tm in angles
        coordslst = lst[0][1:] ## skip tm
        angles    = lst[1][1:] ## skip tm
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

def filterFiles(org_fnames, new_suffix, prefix = None, filter_function = None,
                start = 0, length = 0, zip_filename = None, return_if_exist = True, args = None):
    endidx = start + length
    if prefix is None:
        pre_, fname_ = os.path.split(org_fnames[0])
        fpref_ = ''.join(fname_.split('.')[:-1])
        if pre_ != '':
            pre_ = pre_ + '/'
        prefix = pre_ + fpref_
    new_fname = prefix + '.' + new_suffix
    print('new_fname: {}'.format(new_fname))

    if return_if_exist:
        if zip_filename is None:
            if os.path.exists(new_fname):
                print('Nothing is done because {} already exists'.format(new_fname))
                return
        else:
            try:
                with zipfile.ZipFile(zip_filename, mode='r') as zf:
                    if new_fname in zf.namelist():
                        print('Nothing is done because {} is in {}'.format(new_fname, zip_filename))
                        return
            except Exception as e:
                print('error occured while opening {}'.format(zip_filename))
                raise e

    ## open files
    flst = []
    def _close_func():
        for l in flst:
            l.close()
    if zip_filename is None:
        ## Normal file
        for nm in org_fnames:
            try:
                f = open(nm, 'r')
            except Exception as e:
                print('error occured while opening {}'.format(nm))
                print('{} may not exist.'.format(os.path.basename(nm)))
                _close_func()
                raise e
            flst.append(f)
    else:
        ## ZIP file
        try:
            with zipfile.ZipFile(zip_filename, mode='r') as zf:
                for nm in org_fnames:
                    try:
                        f = zf.open(nm, mode='r')
                    except Exception as e:
                        print('error occured while opening {} in {}'.format(nm, zip_filename))
                        raise e
                    flst.append(f)
        except Exception as e:
            print('error occured while opening {}'.format(zip_filename))
            raise e

    sbuf = io.StringIO() ## compatibility for file and zipfile
    readers = [ csv.reader(f, delimiter=' ').__iter__() for f in flst ]
    writer = csv.writer(sbuf, delimiter=' ',  lineterminator='\n')
    cntr = 0
    ## apply filter
    try:
        ### HOT FIX for python3
        zipfile.ZipExtFile.__next__  = lambda self_: self_.next_original().decode('utf-8')
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
        _close_func()
        zipfile.ZipExtFile.__next__  = zipfile.ZipExtFile.next_original
        raise e
    _close_func()
    zipfile.ZipExtFile.__next__  = zipfile.ZipExtFile.next_original

    ## write data to file
    if zip_filename is None:
        try:
            with open(new_fname, 'w') as fnew:
                fnew.write(sbuf.getvalue())
        except Exception as e:
            print('error occured while opening {}'.format(new_fname))
            print('{} may not exist.'.format(os.path.basename(new_fname)))
            raise e
    else:
        try:
            with zipfile.ZipFile(zip_filename, mode='a') as za:
                za.writestr(new_fname, sbuf.getvalue(), compress_type=za.infolist()[0].compress_type)
        except Exception as e:
            print('error occured while opening {} for writing {}'.format(zip_filename, new_fname))
            raise e
