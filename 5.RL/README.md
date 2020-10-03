The goal of this project was to implement two reinforcement learning methods to teach a NAO robot to kick a penalty goal in a soccer game.  The object was to compare the two methods (Q-learning vs RL-DT) to understand the advantages and disadvantages of each method.

The RL-DT method was based off the following paper:
T. Hester, M. Quinlan, P. Stone: “Generalized Model Learning for Reinforcement Learning on a Humanoid Robot”, IEEE International Conference on Robotics and Automation (ICRA), pp. 2369-2374, 2010.

In the RL-DT method, the agent learned a model of the environment with decision trees to determine action selection (model-based), whereas in the Q-learning approach the agent took actions based on longterm reward with no understanding of the model (model-free approach).  The ball was placed at the penalty position.  The NAO robot held its leg out and would adjust it in and out in order to control the direction of kick toward the goal. Based on vision of where the goalie was, the NAO robot learned to avoid the goalie and kick on the opposite side of the goal to score.

In the RL-DT case, the NAO learned very quickly and performed greedy actions once the model was learned.  The Q-learning agent took longer to learn and required more discrete goalie positions in order to achieve similar results.

