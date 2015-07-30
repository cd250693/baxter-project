# program to find exam statistics
grades = [100, 100, 90, 40, 80, 100, 85, 70, 90, 65, 90, 85, 50.5]


# function to print of grades
def print_grades(grades):
    for i in grades:
        print i


# function to compute the sum of all test grades
def grades_sum(scores):
    total = 0
    for i in scores:
        total += i
    return total


# function to find average of test grades
def grades_average(grades):
    average = grades_sum(grades)
    average = average / float(len(grades))
    return average


# function to compute variance
def grades_variance(scores):
    average = grades_average(scores)
    variance = 0
    for score in scores:
        i = (average - score) ** 2
        variance += i
    variance = variance / len(scores)
    return variance


# function to compute standard deviation
def grades_std_deviation(variance):
    return variance ** 0.5


print_grades(grades)

print grades_sum(grades)

print grades_average(grades)

print grades_variance(grades)

variance = grades_variance(grades)
print grades_std_deviation(variance)