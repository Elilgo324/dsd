import random

import numpy as np
from agents.fixed_velocity_agent import UpwardsAgent
from manimlib import *
from planners.example_planner import ExamplePlanner
from robots.basic_robot import BasicRobot
from environment import Environment
from utils.point import Point
from utils.consts import Consts

global agents, robots


class Ori(Scene):
    def construct(self):
        grid = NumberPlane(Consts.X_RANGE, Consts.Y_RANGE)
        self.add(grid)

        agents = [UpwardsAgent(Point(-2, -2)), UpwardsAgent(Point(2, -2))]
        robots = [BasicRobot(Point(-3, 3)), BasicRobot(Point(3, 3))]

        dot_agents = [Dot(np.array([a.x, a.y, 1]), color=RED) for a in agents]
        dot_robots = [Dot(point=np.array([r.x, r.y, 1]), radius=Consts.DISABLEMENT_RANGE ,color=BLUE) for r in robots]

        agents_dict = dict(zip(agents, dot_agents))
        robots_dict = dict(zip(robots, dot_robots))

        self.add(*dot_agents)
        self.add(*dot_robots)

        env = Environment(agents=agents, robots=robots)
        planner = ExamplePlanner(environment=env)

        is_finished = False
        while not is_finished:
            is_finished = env.advance(debug=True)
            self.wait(0.5)

            for robot in robots:
                robots_dict[robot].move_to(np.array([robot.x,robot.y,1]))

            for agent in agents:
                agents_dict[agent].move_to(np.array([agent.x,agent.y,1]))



class SquareToCircle(Scene):
    def construct(self):
        circle = Circle()  # create a circle
        circle.set_fill(PINK, opacity=0.5)  # set color and transparency

        square = Square()  # create a square
        square.rotate(PI / 4)  # rotate a certain amount

        self.play(Create(square))  # animate the creation of the square
        self.play(Transform(square, circle))  # interpolate the square into the circle
        self.play(FadeOut(square))  # fade out animation


class UpdatersExample(Scene):
    def construct(self):
        random.seed(19)

        num_robots = 3
        num_agents = 6

        grid = NumberPlane()
        self.add(grid)

        square = Square()
        square.set_fill(BLUE_E, 1)

        # On all all frames, the constructor Brace(square, UP) will
        # be called, and the mobject brace will set its data to match
        # that of the newly constructed object
        brace = always_redraw(Brace, square, UP)

        text, number = label = VGroup(
            Text("Width = "),
            DecimalNumber(
                0,
                show_ellipsis=True,
                num_decimal_places=2,
                include_sign=True,
            )
        )
        label.arrange(RIGHT)

        # This ensures that the method deicmal.next_to(square)
        # is called on every frame
        always(label.next_to, brace, UP)
        # You could also write the following equivalent line
        # label.add_updater(lambda m: m.next_to(brace, UP))

        # If the argument itself might change, you can use f_always,
        # for which the arguments following the initial Mobject method
        # should be functions returning arguments to that method.
        # The following line ensures thst decimal.set_value(square.get_y())
        # is called every frame
        f_always(number.set_value, square.get_width)
        # You could also write the following equivalent line
        # number.add_updater(lambda m: m.set_value(square.get_width()))

        self.add(square, brace, label)

        # Notice that the brace and label track with the square
        self.play(
            square.animate.scale(2),
            rate_func=there_and_back,
            run_time=2,
        )
        self.wait()
        self.play(
            square.animate.set_width(5, stretch=True),
            run_time=3,
        )
        self.wait()
        self.play(
            square.animate.set_width(2),
            run_time=3
        )
        self.wait()

        # In general, you can alway call Mobject.add_updater, and pass in
        # a function that you want to be called on every frame.  The function
        # should take in either one argument, the mobject, or two arguments,
        # the mobject and the amount of time since the last frame.
        now = self.time
        w0 = square.get_width()
        square.add_updater(
            lambda m: m.set_width(w0 * math.sin(self.time - now) + w0)
        )
        self.wait(4 * PI)


class Stam(Scene):
    def construct(self):
        random.seed(19)

        num_robots = 3
        num_agents = 6

        grid = NumberPlane()
        self.add(grid)

        # for x in range(-7, 8):
        #     for y in range(-4, 5):        self.add(gr
        #         self.add(Dot(np.array([x, y, 0]), color=DARK_GREY))

        robots = VGroup(*[Dot(np.array([i * (-1) ** i, 3, 0]), color=BLUE) for i in range(num_robots)])
        agents = VGroup(*[Dot(np.array([i * (-1) ** i, -3, 0]), color=RED) for i in range(num_agents)])

        self.add(robots, agents)

        self.wait(1)

        self.play(robots.animate.shift(DOWN * 5),
                  agents.animate.shift(UP * 5), run_time=2)

        self.wait(1)
