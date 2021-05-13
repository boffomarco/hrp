
# https://stonesoup.readthedocs.io/en/latest/auto_tutorials/01_KalmanFilterTutorial.html

import numpy as np
from datetime import datetime, timedelta


from stonesoup.types.groundtruth import GroundTruthPath, GroundTruthState
from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, \
                                               ConstantVelocity

# And the clock starts
start_time = datetime.now()


np.random.seed(1991)


q_x = 0.05
q_y = 0.05
transition_model = CombinedLinearGaussianTransitionModel([ConstantVelocity(q_x),
                                                          ConstantVelocity(q_y)])


truth = GroundTruthPath([GroundTruthState([0, 1, 0, 1], timestamp=start_time)])

num_steps = 20
for k in range(1, num_steps + 1):
    truth.append(GroundTruthState(
        transition_model.function(truth[k-1], noise=True, time_interval=timedelta(seconds=1)),
        timestamp=start_time+timedelta(seconds=k)))


from stonesoup.plotter import Plotter
plotter = Plotter()
plotter.plot_ground_truths(truth, [0, 2])

plotter.fig

print(transition_model.matrix(time_interval=timedelta(seconds=1)))

print(transition_model.covar(time_interval=timedelta(seconds=1)))
