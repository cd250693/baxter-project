# -*- coding: utf-8 -*-
# example list sent from the vision system
# colours are:
#            o = orange
#            n = blue
#            r = red
#            g = green
#            w = white
#            y = yellow
facevs = ('rnwgrwono', 'nwgyoowyr', 'roygynyyn',
          'grwwwgwrg', 'ywngnygny', 'roorgonro')


# tests to help check if the vision system worked correctly
def basicTests(facecode):
    # assigns each list entry to its corresponding face variable
    f = facecode[0]
    b = facecode[1]
    u = facecode[2]
    d = facecode[3]
    l = facecode[4]
    r = facecode[5]
    # test if each face string has 9 colours
    if any(len(x) is not 9 for x in (f, b, u, d, l, r)):
        print('not nine colours in each face')
        return False
    # test if there is 9 of each colour
    colourtest = f + b + u + d + l + r
    numO = colourtest.count('o')
    numN = colourtest.count('n')
    numR = colourtest.count('r')
    numG = colourtest.count('g')
    numW = colourtest.count('w')
    numY = colourtest.count('y')
    print(numO, numN, numR, numG, numW, numY)
    if any(x is not 9 for x in (numO, numN, numR, numG, numW, numY)):
        print('not nine of each colour')
        return False
    # test if each centre colour is unique
    centres = f[4] + b[4] + u[4] + d[4] + l[4] + r[4]
    cenO = centres.count('o')
    cenN = centres.count('n')
    cenR = centres.count('r')
    cenG = centres.count('g')
    cenW = centres.count('w')
    cenY = centres.count('y')
    print(cenO, cenN, cenR, cenG, cenW, cenY)
    if any(x is not 1 for x in (cenO, cenN, cenR, cenG, cenW, cenY)):
        print('centre colours are invalid')
        return False
    #all tests passed
    return True

print(basicTests(facevs))
