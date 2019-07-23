import easygui

a = easygui.fileopenbox(filetypes = ['*.m'])

f = open('tmp', 'w')
if isinstance(a, basestring):
    f.write(a)
f.close()
