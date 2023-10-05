import numpy as np
import csv
import os
import io
import zipfile
zipfile.ZipExtFile.next_original = zipfile.ZipExtFile.__next__
##
import irsl_choreonoid.robot_util as ru
from cnoid.IRSLCoords import coordinates

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

def create_writeLinkPositionFunction(_robot, link_index=0, link_name=None, offset=None):
    _link_idx = link_index
    if type(link_name) == str:
        _link_idx = _robot.links().index(_robot.link(link_name))
    ## def closure_func__(coordslst, angles): ## coordslst[pos(3), rot(9)] joint_angles(N)
    def closure_func__(lst, **kwargs): ## coordslst[pos(3), rot(9)] joint_angles(N)
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
        if offset:
            cds.transform(offset)
        return tm + cds.pos.tolist() + cds.getRPY().tolist()
    return closure_func__

def create_writeCOGPositionFunction(_robot, link_index=0):
    ## def closure_func__(coordslst, angles): ## coordslst[pos(3), rot(9)] joint_angles(N)
    def closure_func__(lst, **kwargs): ## coordslst[pos(3), rot(9)] joint_angles(N)
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

#### Class modules for parsing hrpsys-log
class GroundTruthInfo(object):
    def __init__(self, base, robot_name, zipfile=None, modelfile=None):
        self.robot = None
        self.base = base
        self.suffix = ['{}(Robot)0_WAIST'.format(robot_name),
                       'RobotHardware_choreonoid0_q']
        self.names = [ base + '.' + s for s in self.suffix ]

        if len(zipfile) == 0 or zipfile is None:
            self.zipfile = None
        else:
            self.zipfile = zipfile

        if modelfile is not None:
            self.loadRobot(modelfile)

    def loadRobot(self, fname):
        self.robot = ru.loadRobot(fname)
        if self.robot is None:
            raise Exception('load body fail : {}'.format(fname))

class ProcessBase(object):
    def __init__(self, info, modelfile=None, suffix=None):
        self.info = info
        self.suffix=suffix
        self.func = None
        if modelfile is not None:
            self.info.loadRobot(modelfile)
        if self.info.robot is not None:
            self.createFunction()

    def createFunction(self):
        print('call : {}'.format(self))

    def process(self, fname=None, return_if_exist=True):
        if fname:
            self.info.loadRobot(fname)
            self.createFunction()
        if self.func is None:
            if self.info.robot is not None:
                self.createFunction()
            else:
                raise Exception('')

        filterFiles(self.info.names, self.suffix, zip_filename=self.info.zipfile, return_if_exist=return_if_exist,
                    filter_function = self.func)

class ProcessCOG(ProcessBase):
    def __init__(self, info, modelfile=None, suffix='pos_COG'):
        super().__init__(info, modelfile=modelfile, suffix=suffix)

    def createFunction(self):
        self.func = create_writeCOGPositionFunction(self.info.robot)

class ProcessEEF(ProcessBase):
    def __init__(self, info, modelfile=None, suffix='pose_eef', link_name=None, offset=None):
        self.link_name = link_name
        self.offset = offset
        super().__init__(info, modelfile=modelfile, suffix=suffix)

    def createFunction(self):
        self.func = create_writeLinkPositionFunction(self.info.robot, link_name=self.link_name, offset=self.offset)

## new class
class HrpsysLogFiles(object):
    def __init__(self, file_names, zip_filename=None):
        self.file_names = file_names
        self.zip_filename = zip_filename

        ## open files
        self.file_list = []
        self.open_files()

    def close(self):
        for l in self.file_list:
            l.close()
    def open_files(self):
        self.close()
        if self.zip_filename is None:
            ## Normal file
            for nm in self.file_names:
                try:
                    f = open(nm, 'r')
                except Exception as e:
                    print('error occured while opening {}'.format(nm))
                    print('{} may not exist.'.format(os.path.basename(nm)))
                    self.close()
                    raise e
                self.file_list.append(f)
        else:
            ## ZIP file
            try:
                with zipfile.ZipFile(self.zip_filename, mode='r') as zf:
                    for nm in self.file_names:
                        try:
                            f = zf.open(nm, mode='r')
                        except Exception as e:
                            print('error occured while opening {} in {}'.format(nm, self.zip_filename))
                            raise e
                        self.file_list.append(f)
            except Exception as e:
                print('error occured while opening {}'.format(self.zip_filename))
                raise e
    def applyFunction(self, functions, start=0, length=0, skip=1, process_result_functions=None):
        sbuf = io.StringIO() ## compatibility for file and zipfile
        readers = [ csv.reader(f, delimiter=' ').__iter__() for f in self.file_list ]
        endidx = start + length
        cntr = 0
        skip_counter = 0
        if type(functions) is not list and type(functions) is not tuple:
            functions = [ functions ]
        if process_result_functions is not None and type(process_result_functions) is not list and type(process_result_functions) is not tuple:
            process_result_functions = [ process_result_functions ]
        elif process_result_functions is None:
            process_result_functions = [ None ] * len(functions)
        try:
            ### HOT FIX for python3
            zipfile.ZipExtFile.__next__  = lambda self_: self_.next_original().decode('utf-8')
            for rows in zip(*readers):
                if cntr >= start and skip_counter >= skip: ###
                    # dls = [ filter(lambda x: x != '', row) for row in rows ] ## python2
                    dls = [ [ float(x) for x in filter(lambda x: x != '', row) ] for row in rows ] ## python3
                    for func, res_func in zip(functions, process_result_functions):
                        res = func(dls)
                        if res_func is not None:
                            res_func(res)
                    skip_counter = 0
                skip_counter += 1
                cntr += 1
                if cntr == endidx:
                    break
        except Exception as e:
            print('error occured while reading data : {}'.format(e))
            self.close()
            zipfile.ZipExtFile.__next__  = zipfile.ZipExtFile.next_original
            raise e
        self.close()
        zipfile.ZipExtFile.__next__  = zipfile.ZipExtFile.next_original
