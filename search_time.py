import sys

time = float(sys.argv[1])
for line in open("vesc.txt"):
    l = eval(line)
    if l[0] >= time:
        print(line)
        break

