# -*- coding: utf-8 -*-


def formlist():
    my_list = [i ** 2 for i in range(1, 11)]
# Generates a list of squares of the numbers 1 - 10
    f = open('outputlist.txt', 'w')
    for item in my_list:
        f.write(str(item) + '\n')
    f.close()


def fileIO():
    myfile = open('output.txt', 'r+')
    my_list = [i ** 2 for i in range(1, 11)]
    for i in my_list:
        myfile.write(str(i) + '\n')
    myfile.close()


def fileRead():
    my_file = open('output.txt', 'r')
    print(my_file.read())
    my_file.close()


def fileReadLine():
    f = open('text.txt', 'r')
    print f.readline()
    print f.readline()
    print f.readline()
    f.close()


def UseWithAs():
    with open('text.txt', 'w') as textfile:
        textfile.write('Success!')
    # to test whether the file has been closed properly
    if textfile.closed is not True:
        textfile.close()
    print textfile.closed
