# -*- coding: utf-8 -*-


class Car(object):

    condition = 'New'

    def __init__(self, model, colour, mpg):
        self.model = model
        self.colour = colour
        self.mpg = mpg

    def display_car(self):
        print('This is a ' + self.colour + ' ' +
        self.model + ' with ' + str(self.mpg) + ' MPG.')

    def drive_car(self):
        self.condition = 'Used'


class ElectricCar(Car):

    def __init__(self, model, colour, mpg, battery_type):
        self.model = model
        self.colour = colour
        self.mpg = mpg
        self.battery_type = battery_type

    def drive_car(self):
        self.condition = 'like new'


my_car = Car('DeLorean', 'Silver', 88)
print my_car.display_car()
print my_car.condition
my_car.drive_car()
print my_car.condition

my_car2 = ElectricCar('Tesla', 'Electric Blue', 1000, 'Molten Salt')
print my_car2.display_car()
my_car2.drive_car()
print my_car2.condition