# from robots.base_robot import Chaser
# from agents.base_agent import Chaseable
#
#
# class DPRobot(Chaser):
#     def __init__(self, x, y, fv, r=0):
#         super().__init__(x, y, fv, r)
#         self._T = None
#         self._cur_state = Chaseable(x, y)
#         self._direction = 'up'
#
#     def _create_table(self, chaseables):
#         X = sorted([c for c in chaseables if c.y >= self._y], key=lambda c: c.y)
#         Y = sorted([c for c in chaseables if c.y < self._y], reverse=True, key=lambda c: c.y)
#
#         X = [self._cur_state] + X
#         Y = [self._cur_state] + Y
#
#         T = {c1: {c2: {'t': None, 'damage': None, 'kill': None} for c2 in X + Y} for c1 in X + Y}
#         T[self._cur_state][self._cur_state] = {'t': 0, 'damage': 0, 'kill': self._cur_state}
#
#         for i in range(1, max(len(X), len(Y))):
#             for j in range(0, max(len(X), len(Y))):
#                 num_living_locust = len(chaseables) + 1 - (i + j)
#
#                 if i < len(X) and j < len(Y):
#                     time_reaching_from_x = self._time_to_meet(X[i - 1], X[i])
#                     time_reaching_from_y = self._time_to_meet(Y[j], X[i])
#
#                     damage_reaching_from_x = T[X[i - 1]][Y[j]]['damage'] + num_living_locust * time_reaching_from_x
#                     damage_reaching_from_y = T[Y[j]][X[i - 1]]['damage'] + num_living_locust * time_reaching_from_y
#
#                     if damage_reaching_from_x < damage_reaching_from_y:
#                         T[X[i]][Y[j]] = {'t': time_reaching_from_x, 'damage': damage_reaching_from_x, 'kill': X[i]}
#                     else:
#                         T[X[i]][Y[j]] = {'t': time_reaching_from_y, 'damage': damage_reaching_from_y, 'kill': X[i]}
#
#                 if j < len(X) and i < len(Y):
#                     time_reaching_from_x = self._time_to_meet(X[j], Y[i])
#                     time_reaching_from_y = self._time_to_meet(Y[i - 1], Y[i])
#
#                     damage_reaching_from_x = T[X[j]][Y[i - 1]]['damage'] + num_living_locust * time_reaching_from_x
#                     damage_reaching_from_y = T[Y[i - 1]][X[j]]['damage'] + num_living_locust * time_reaching_from_y
#
#                     if damage_reaching_from_x < damage_reaching_from_y:
#                         T[Y[i]][X[j]] = {'t': time_reaching_from_x, 'damage': damage_reaching_from_x, 'kill': Y[i]}
#                     else:
#                         T[Y[i]][X[j]] = {'t': time_reaching_from_y, 'damage': damage_reaching_from_y, 'kill': Y[i]}
#         print(T)
#         print(T[X[-1]][Y[-1]])
#         print(T[Y[-1]][X[-1]])
#         self._T = T
#
#     def _time_to_meet(self, source, target):
#         if source.y < target.y:
#             delta_y = target.y - source.y
#             time_to_meet = delta_y / (self._fv + target.v)
#         else:
#             delta_y = source.y - target.y
#             time_to_meet = delta_y / (self._fv - target.v)
#
#         return time_to_meet
#
#     def calc_t(self, chaseables):
#         if not self._T:
#             self._create_table(chaseables)
#
#         t, k = self._T[self._cur_state][self._cur_state]['t'], self._T[self._cur_state][self._cur_state]['kill']
#
#         if k.y >= self._y:
#             self._direction = 'up'
#         else:
#             self._direction = 'down'
#
#         return t, k
#
#     def advance(self, t):
#         if self._direction == 'up':
#             self._y += t * self._fv
#         else:
#             self._y -= t * self._fv
#
#     def __str__(self):
#         return 'DPRobot'
