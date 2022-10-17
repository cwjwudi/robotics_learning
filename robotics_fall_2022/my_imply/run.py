from manipulato_2D import Manipulator_2D_fk

if __name__ == '__main__':
    env = Manipulator_2D_fk()
    action = 0
    for i in range(10000):
        env.step(action)
