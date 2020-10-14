import pty
import os
import select

def mkpty():
    master1, slave = pty.openpty()
    slaveName1 = os.ttyname(slave)
    #master2, slave = pty.openpty()
    #slaveName2 = os.ttyname(slave)
    print ('\nslave device name: ', slaveName1)
    return master1#, master2
 
if __name__ == "__main__":
    master1 = mkpty()
    while True:
        rl, wl, el = select.select([master1], [], [], 1)
        data = os.read(master1, 128)
        print ("read %d data." % len(data))
        print ("which is", data)
        os.write(master1, data)
