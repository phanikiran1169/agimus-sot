from dynamic_graph.sot.hpp import PathSampler
ps = PathSampler ('ps')
ps.loadRobotModel ('sot_youbot', 'planar', 'pr2_sot')
q_init = 36*[0]
q_init [0] = 1
q_init [1] = 1
q_init [2] = 1
q_end = q_init [::]
q_end [34] = 2
ps.addWaypoint (tuple (q_init))
ps.addWaypoint (tuple (q_end))
ps.configuration.recompute (0)
ps.setTimeStep (0.1)
ps.start ()
ps.configuration.value
for i in range (11):
    ps.configuration.recompute (i)
    print (ps.configuration.value [0])

ps.resetPath ()

for i in range (111, 121):
    ps.configuration.recompute (i)
    #print (ps.configuration.value [0])

ps.addWaypoint (tuple (q_init))

for i in range (121, 131):
    ps.configuration.recompute (i)
    #print (ps.configuration.value [0])

ps.start ()

for i in range (131, 240):
    ps.configuration.recompute (i)
    #print (ps.configuration.value [0])
