import pybullet_envs
import gym
import numpy as np
from sac import SAC_agent
from env3 import ArmEnv
from utils import plot_learning_curve
from gym import wrappers


# if ON_TRAIN:
#     Mode = 'Train'
# else:
#     Mode = 'Eval'

# env = ArmEnv('Train')
# max_action = [np.pi / 2, np.pi]
#InvertedPendulumBulletEnv-v0
# env = gym.make('Pendulum-v0')
# env = gym.make('Pendulum-v0')
#
# input_dims = env.observation_space.shape[0]
# n_actions = env.action_space.shape[0]
# max_action = env.action_space.high[0]
# agent = SAC_agent(input_dims, n_actions, max_action)


def train():
    env = gym.make('Pendulum-v0')
    env.reset()
    env.render()



    input_dims = env.observation_space.shape[0]

    n_actions = env.action_space.shape[0]

    max_action = env.action_space.high[0]
    print('max_action',max_action)

    agent = SAC_agent(input_dims, n_actions, max_action)
    n_games = 250
    # uncomment this line and do a mkdir tmp && mkdir video if you want to
    # record video of the agent playing the game.
    #env = wrappers.Monitor(env, 'tmp/video', video_callable=lambda episode_id: True, force=True)
    filename = 'inverted_pendulum.png'

    figure_file = 'plots/' + filename

    best_score = env.reward_range[0]
    print(best_score)
    score_history = []
    load_checkpoint = False

    if load_checkpoint:
        agent.load_models(PATH_EVAL)
        # env.render(mode='human')

    for i in range(n_games):
        observation = env.reset()
        done = False
        score = 0
        steps = 0
        for j in range(500):
            env.render()
            action = agent.choose_action(observation)
            #print('action',action)
            observation_, reward, done,info = env.step(action)
            # print('reward',reward)
            steps += 1
            agent.remember(observation, action, reward, observation_, done)
            # if not load_checkpoint:
            #     agent.learn()
            score += reward
            observation = observation_
            agent.learn()

            if done or j == 499:
                print('episode ', i, 'score %.1f' % score,  'steps:%i' % j)
                break



        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        #
        # if avg_score > best_score:
        #     best_score = avg_score
        #     if not load_checkpoint:
        #         agent.save_models()
        #print( 'avg_score %.1f' % avg_score )
        print('episode ', i, 'score %.1f' % score, 'avg_score %.1f' % avg_score,'steps:%i'%steps,)

        if i % 10 == 0:
            print('i',i % 10)
            agent.save_models()  # save model

    if not load_checkpoint:
        x = [i+1 for i in range(n_games)]
        plot_learning_curve(x, score_history, figure_file)


# def train():
#     # env = gym.make('InvertedPendulumBulletEnv-v0')
#     # agent = Agent(input_dims=env.observation_space.shape, env=env,
#     #         n_actions=env.action_space.shape[0])
#     n_games = 250
#     # uncomment this line and do a mkdir tmp && mkdir video if you want to
#     # record video of the agent playing the game.
#     #env = wrappers.Monitor(env, 'tmp/video', video_callable=lambda episode_id: True, force=True)
#     filename = 'inverted_pendulum.png'
#
#     figure_file = 'plots/' + filename
#
#     best_score = env.reward_range[0]
#     score_history = []
#     load_checkpoint = False
#
#     if load_checkpoint:
#         agent.load_models()
#         env.render(mode='human')
#
#     for i in range(n_games):
#         observation = env.reset()
#         done = False
#         score = 0
#         while not done:
#             action = agent.choose_action(observation)
#             observation_, reward, done, info = env.step(action)
#             # print('reward',reward)
#             score += reward
#             agent.remember(observation, action, reward, observation_, done)
#             if not load_checkpoint:
#                 agent.learn()
#             observation = observation_
#         score_history.append(score)
#         avg_score = np.mean(score_history[-100:])
#
#         if avg_score > best_score:
#             best_score = avg_score
#             if not load_checkpoint:
#                 agent.save_models()
#
#         print('episode ', i, 'score %.1f' % score, 'avg_score %.1f' % avg_score)
#
#     if not load_checkpoint:
#         x = [i+1 for i in range(n_games)]
#         plot_learning_curve(x, score_history, figure_file)


if __name__ == "__main__":

    train()