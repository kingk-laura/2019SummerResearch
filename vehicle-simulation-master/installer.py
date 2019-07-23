
#!python
# coding: utf-8
import sys
import ctypes
import os
import subprocess

def run_as_admin(argv=None, debug=False):
    shell32 = ctypes.windll.shell32
    if argv is None and shell32.IsUserAnAdmin():
        return True

    if argv is None:
        argv = sys.argv
    if hasattr(sys, '_MEIPASS'):
        # Support pyinstaller wrapped program.
        arguments = map(unicode, argv[1:])
    else:
        arguments = map(unicode, argv)
    argument_line = u' '.join(arguments)
    executable = unicode(sys.executable)
    if debug:
        print 'Command line: ', executable, argument_line
    ret = shell32.ShellExecuteW(None, u"runas", executable, argument_line, None, 1)
    if int(ret) <= 32:
        return False
    return None


if __name__ == '__main__':
    ret = run_as_admin()
    if ret is True:
        print 'I have admin privilege.'
        os.system("set path=%PATH%;%PYTHONPATH%;")
        path = os.environ.get('path')
        pathlist = path.split(';')
        matlabpath = min([s for s in pathlist if 'MATLAB' in s], key=len) 
        print(matlabpath)
        os.chdir(matlabpath+"/../extern/engines/python")
        print(os.getcwd())
        os.system('python setup.py build --build-base=%TEMP% install')
        os.system('python -m pip install docutils pygments pypiwin32 kivy.deps.sdl2 kivy.deps.glew')
        os.system('python -m pip install cython kivy numpy matplotlib')
        
        raw_input('Press ENTER to exit.')
    elif ret is None:
        print 'I am elevating to admin privilege.'
        raw_input('Press ENTER to exit.')
    else:
        print 'Error(ret=%d): cannot elevate privilege.' % (ret, )
