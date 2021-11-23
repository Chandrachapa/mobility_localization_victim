import xpress as xp
import math
#TO INSTALL XPRESS: RUN 'pip install xpress' 
x1 = xp.var()
x2 = xp.var()
x3 = xp.var()
y1 = xp.var()
y2 = xp.var()
y3 = xp.var()

p = xp.problem()

p.addVariable(x1)
p.addVariable(x2)
p.addVariable(x3)
p.addVariable(y1)
p.addVariable(y2)
p.addVariable(y3)

# polynomial constraint
p.addConstraint((x2-1)**2 + (y2-3)**2 -1 <= 0)
p.addConstraint(3<=((x1-x2)**2 + (y1-y2)**2)**0.5  <= 5)
p.addConstraint((x1-5)**2 + (y1-5)**2 -4 <= 0)
p.addConstraint((x3-10)**2 + (y3-1)**2 -1 <= 0)
p.addConstraint(4<=((x1-x3)**2 + (y1-y3)**2)**0.5 <= 7)
p.addConstraint(3<=((x1-x2)**2 + (y1-y2)**2)**0.5 <= 5)
p.setObjective(1)

p.solve()

print('solution: ', p.getSolution(x1,y1), '; value: ',  p.getSolution(x2,y2), p.getSolution(x3,y3))
