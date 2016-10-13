import re
import subprocess

class OrScreenshotter:
    """
    A class that takes a screenshot of a running OpenRAVE viewer simulation window.
    Originally written by Chris Dellin.
    -- Might screenshot any open OpenRAVE viewer window, so make sure that you have
        only one running when initializing OrScreenshotter
    -- Will screenshot the literal window that is open, so the larger you have the
        window on your screen, the better resolution the pictures will be.
    -- You should specify format (.png works well) when calling the capture
        function. By default, saves as .ps which looks pretty bad.
    -- In the init function, we look to find the window ID of the viewer window.
        This does not always work as the way window info is formatted seems to not
        be totally consistent. Sometimes on machines where this works, I get the
        "couldn't find windowid!" error. I have sometimes fixed this by
        restarting the computer but we should find a better fix to this.
    """
    def __init__(self, env):
        self.env = env
        # get windowid
        window_name = "OpenRAVE"
        alternate_window_name = ' +(0x[0-9a-fA-F]+) +"{}"'.\
            format(self.env.GetViewer().GetName())
        found = []
        for line in subprocess.check_output(['xwininfo','-root','-tree']).\
                               decode('utf8').split('\n'):
            m = re.search(window_name, line)
            if m is not None:
                found.append(line.split()[0])
            # This is the search and append in Chris's init function. In my
            # experience, the first window_name is what Ubuntu 14.04 calls openrave
            # viewer windows, but maybe this is not constant between machines.
            m = re.search(alternate_window_name, line)
            if m is not None:
                found.append(m.group(1))
            
        if len(found) != 1:
            raise RuntimeError('couldnt find windowid!')
        self.windowid = found[0]

    def capture(self,fname,zoom=1.0):
        # collect window statistics
        stats = {}
        for line in subprocess.check_output(['xwininfo','-id',self.windowid]).\
                               decode('utf8').split('\n'):
            array = line.split(':',1)
            if len(array) != 2:
                continue
            stats[array[0].strip()] = array[1].strip()
        # width is minus chrome
        w = int(stats['Width']) - 42 - 30
        h = int(stats['Height']) - 23 - 52
        l = int(stats['Absolute upper-left X']) + 42
        t = int(stats['Absolute upper-left Y']) + 23
        wz = int(w / zoom)
        hz = int(h / zoom)
        lz = l + int((w-wz)/2)
        tz = t + int((h-hz)/2)
        cmd = ['import','-window','root','-crop',
           '{}x{}+{}+{}'.format(wz,hz,lz,tz),
           fname]
        subprocess.check_call(cmd)
