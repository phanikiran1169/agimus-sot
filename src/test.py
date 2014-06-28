from dynamic_graph.sot.hpp import PathSampler
ps = PathSampler ('ps')
ps.loadRobotModel ('hpp_tutorial', 'planar', 'pr2')
q_init = 47*[0]
q_init [3] = 1
q_end = q_init [::]
q_end [0] = 1
ps.addWaypoint (tuple (q_init))
ps.addWaypoint (tuple (q_end))

ps.configuration.recompute (0)
ps.setTimeStep (.01)
ps.start ()

for i in range (110):
    ps.configuration.recompute (i)
    print (ps.configuration.value [0])

ps.resetPath ()

for i in range (111, 121):
    ps.configuration.recompute (i)
    print (ps.configuration.value [0])

ps.addWaypoint (tuple (q_init))

for i in range (121, 131):
    ps.configuration.recompute (i)
    print (ps.configuration.value [0])

ps.start ()

for i in range (131, 240):
    ps.configuration.recompute (i)
    print (ps.configuration.value [0])
