import errno
import os
from datetime import datetime

def createLogDir():
    mace_root_dir = os.environ['MACE_ROOT']
    file_prefix = '/MaceSetup_Configs/SimEnvironment/TestConfigs/'
    logDir = os.path.join(file_prefix, datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    print('Log directory is: ' + logDir)
    try:
        os.makedirs(mace_root_dir + logDir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise  # This was not a "directory exist" error..

    return logDir

print(createLogDir())
