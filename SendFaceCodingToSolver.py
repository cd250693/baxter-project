############## Alternative to comunicate with the web server ##############
# import urllib2
# colourencoding = 'bdrfuululululrddrubbflfbdbbfdrrdbdurlrudlffurfrdfblbfl'
# data = urllib2.urlopen('http://127.0.0.1:8081/?' + colourencoding).read()
# print(data)

import requests
import subprocess

#subprocess.Popen('Solver/cube512htm.exe')

# variable stores face encoding in Singmaster Notation
FaceCode = 'BDRFUULULULULRDDRUBBFLFBDBBFDRRDBDURLRUDLFFURFRDFBLBFL'

### changed to use requests, changed over to the new error raised when having a connection error ###
try:
    # send data to localhost 127.0.0.1 port 8081
    r = requests.get('http://127.0.0.1:8081/?' + FaceCode)
except requests.exceptions.ConnectionError:
    print('Cannot connect')


# save data retrieved from the webserver
data = r.content

# remove HTML encodings, carrige returns and new line pointers
data = data.replace('<HTML><BODY>\r\n', '')
data = data.replace('\r\n</BODY></HTML>\r\n', '')
print('------------')
print(data)
print('------------')

# clears Cube Explorer's main window
requests.get('http://127.0.0.1:8081/?' + 'clear')

# Breaks string into a list of individual moves
# this can be done because moves are seperated by whitespace
# Moves are saved as a list under the variable man
# By not defining an argument,
# consecutive whitespace is counted as one seperation
man = data.split()
print(len(man))
for i in man:
    print(i)