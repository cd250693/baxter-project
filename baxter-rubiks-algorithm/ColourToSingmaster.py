# variables obtained by the vision system
# colours are: o = orange, n = blue, r = red, g = green, w = white, y = yellow
# do not match colour letter and face letter, causes .replace issues
front = 'onrgwngrg'
back  = 'nwngyrnno'
up    = 'rorrgywyg'
down  = 'oyyynnywg'
left  = 'wgnorwogw'
right = 'woyrowroy'

# gets the center colour of each face
f = front[4]
b = back[4]
u = up[4]
d = down[4]
l = left[4]
r = right[4]
print (f,b,u,d,l,r)
FaceCode = up + right + front + down + left + back

print (FaceCode)
# replaces colour letter with its corresponding face letter
FaceCode = FaceCode.replace(f, 'f').replace(b, 'b').replace(u, 'u')
FaceCode = FaceCode.replace(d, 'd').replace(l, 'l').replace(r, 'r')

print (FaceCode)