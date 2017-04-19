import math
import sys

a = 2.1244

aa = str("%.2f" % round(a,2))

b = 10.94241243124

if b > 9.99:
    b = 9.99

bb = str("%.2f" % round(b,2))

c = 22
d = 201
e = 1

cc = str("%03d" % c)
dd = str("%03d" % d)
ee = str("%03d" % e)
print(aa +',' + bb + ',' + cc + ',' + dd +',' + ee)