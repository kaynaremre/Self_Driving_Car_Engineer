p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
Z = 'red'
pHit = 0.6
pMiss = 0.2

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    sum_q = sum(q)
    q = [a/sum_q for a in q]
    return q

def move(p, U):
    U = U % len(p)
    q = p[-U:] + p[:-U]

print (sense(p,Z))