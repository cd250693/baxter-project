score = {"a": 1, "c": 3, "b": 3, "e": 1, "d": 2, "g": 2,
         "f": 4, "i": 1, "h": 4, "k": 5, "j": 8, "m": 3,
         "l": 1, "o": 1, "n": 1, "q": 10, "p": 3, "s": 1,
         "r": 1, "u": 1, "t": 1, "w": 4, "v": 4, "y": 4,
         "x": 8, "z": 10}
import string


# function to add up the scrabble score for a word. scores are within score
def scrabble_score(word):
    word = string.lower(word)
    x = 0
    total = 0
    while x < len(word):
        total += score[word[x]]
        x += 1
    return total

print scrabble_score('Pie')


# function that takes a string of text and a word. The word is replaced
# by asterixes within the text, then returned.
def censor(text, word):
    text = text.split()
    cen = []
    for w in text:
        if w == word:
            cen.append('*' * len(w))
        else:
            cen.append(w)
    cen = ' '.join(cen)
    return cen

print censor('hey hey hey', 'hey')


# a fucntion to count the number of time an item appears in sequence
def count(sequence, item):
    num = 0
    for i in sequence:
        if i == item:
            num += 1
    return num

print count([1, 2, 1, 1], 1)


# function to filter a list. All even numbers are returned in a new list
def purify(nums):
    pure = []
    for i in nums:
        if i % 2 == 0:
            pure.append(i)
    return pure

print purify([1,2,3])


# multiplied all numbers input
def product(nums):
    total = 1
    for i in nums:
        total *= i
    return total

print product([4,5,5])


# function to remove elements that are the same
def remove_duplicates(l):
    dupe = []
    for i in l:
        if i not in dupe:
            dupe.append(i)
    return dupe

print remove_duplicates([1, 1, 2, 2, 3, 3, 5, 3])


# function to find the median of a list
def median(l):
    mednum = 0
    l = sorted(l)
    if len(l) % 2 == 1:
        mednum = l[len(l) / 2]
    else:
        num1 = l[len(l) / 2 - 1]
        num2 = l[len(l) - len(l) / 2]
        mednum = (num1 + num2) / 2.0
    return mednum

print median([5, 10, 5, 11, 2, 6, 3, 7, 8, 10, 1, 4])