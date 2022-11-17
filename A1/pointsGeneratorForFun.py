import itertools
maxRange = 2500
x = range(0,maxRange)
aList =[]
for pair in itertools.combinations(x,2):
    for i in range(0,maxRange):
        aList.append(pair)
print(aList)