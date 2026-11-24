"""Extract non-overlapping direct child costs at one level of each loaded-frame call tree."""
from collections import Counter
import csv
import gzip
from pathlib import Path
import re

root=Path(__file__).resolve().parent
def parse(path):
    names={}
    calls,work,self_work=Counter(),Counter(),Counter()
    caller=callee=waiting=None
    for line in gzip.open(path,"rt"):
        if match:=re.match(r"(c?fn)=\((\d+)\)(?: (.*))?",line):
            kind,number,name=match.groups()
            if name: names[number]=name
            if kind=="fn": caller=number
            else: callee=number
        elif line.startswith("calls="):
            waiting=int(line.split()[0][6:])
        elif re.match(r"(?:[+\-]?\d+|\*) ",line):
            instructions=int(line.split()[1])
            if waiting is None: self_work[caller]+=instructions
            else:
                edge=(caller,callee)
                calls[edge]+=waiting
                work[edge]+=instructions
                waiting=None
    return names,calls,work,self_work

if __name__=="__main__":
    variants={"kinetica-coupled":("kp::World::step(", "kp::SoftStep::step(", "kp::(anonymous namespace)::LiveContacts::solveAndMove("),
              "box3d":("b3World_Step",), "rp3d":("reactphysics3d::PhysicsWorld::update(",)}
    with (root/"phase-tree.csv").open("w",newline="") as dest:
        w=csv.writer(dest)
        w.writerow(["variant","parent","child","Ir","pct_of_parent"])
        for variant,parents in variants.items():
            names,calls,edges,own=parse(root/variant/"callgrind.out.gz")
            for prefix in parents:
                matches=[i for i,n in names.items() if n.startswith(prefix)]
                assert len(matches)==1,(prefix,matches)
                i=matches[0]
                costs=[(names[b],cost) for (a,b),cost in edges.items() if a==i]
                costs.append(("[self]",own[i]))
                total=sum(count for _,count in costs)
                for child,count in sorted(costs,key=lambda x:-x[1]):
                    w.writerow([variant,names[i],child,count,f"{100*count/total:.4f}"])
