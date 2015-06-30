from subprocess import Popen, PIPE
import commands



# Checks if Cube Solver is running
output = commands.getoutput('ps -A')
if 'cube512htm' in output:
    print "ITS ALIVE!!!"
else:
    out = 0
    print 'fixing that...'
    # Runs Cube Solver
    solver = Popen(['Solver/cube512htm.exe'], stdin=PIPE, stdout=out, stderr=PIPE)
    print 'done!'
    print out


# to exit Cube solver
def exitSolver():
    subprocess.Popen.kill()
    print 'exiting'
    subprocess.Popen.wait()