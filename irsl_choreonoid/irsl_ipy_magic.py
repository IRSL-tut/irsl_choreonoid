from IPython.core.magic import register_line_magic
import IPython.display
import irsl_choreonoid.cnoid_base as ib

@register_line_magic
def display(line):
    print(line)
    _fname = "/tmp/tmp.png" if len(line) < 4 else line
    ib.saveImageOfScene(_fname)
    _img = IPython.display.Image(_fname)
    IPython.display.display(_img)
