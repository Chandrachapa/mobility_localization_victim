C_v = [1.,1.,1.,1.,0.1,0.1,0.1,0.1]#weights
n = 10

with Model() as M:
  x = M.variable([n,1], Domain.binary())

  M.constraint(Expr.sum(x,0), Domain.greaterThan(1.0))#minimum constraints selected : greater than 1

  #run csp algorithm function, return res 0 or 1
  res = 1.0
  #M.constraint(res,  Domain.equalsTo(1.))