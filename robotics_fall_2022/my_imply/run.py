from manipulato_2D import Manipulator_2D_fk, Manipulator_2D_ik

if __name__ == '__main__':
    env = Manipulator_2D_ik()
    action = 0
    for i in range(10000):
        done = env.step(action)
        if done:
            break
