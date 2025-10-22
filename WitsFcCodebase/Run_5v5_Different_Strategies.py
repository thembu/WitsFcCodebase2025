# Run_5v5_Different_Strategies.py
from scripts.commons.Script import Script
script = Script()
a = script.args

from agent.Agent import Agent
from agent.Agent_Opponent import Agent as Agent_Opponent

# Your team with custom strategy (with prints)
team_args = ((a.i, a.p, a.m, u, a.t, True, True) for u in range(1,6))
script.batch_create(Agent, team_args)

# Opponent team with default strategy (no prints)
team_args = ((a.i, a.p, a.m, u, "Opponent", True, False) for u in range(1,6))
script.batch_create(Agent_Opponent, team_args)

while True:
    script.batch_execute_agent()
    script.batch_receive()