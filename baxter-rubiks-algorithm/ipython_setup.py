from baxter_rubiks import Baxter
import logging

logger = logging.getLogger('baxterRubiks')

baxter = Baxter()
logger.setLevel('DEBUG')
logger.handlers[0].setLevel('DEBUG')

print('class intialized as baxter')
print('ready to go')


# from baxter_rubiks import CubeExplorer
# explorer = CubeExplorer()

# print('initialized as explorer')
# print('ready to go')

# from baxter_rubiks import CubeExplorer
# import logging

# cube = CubeExplorer()

# print('class initialized as cube\nready to go')
