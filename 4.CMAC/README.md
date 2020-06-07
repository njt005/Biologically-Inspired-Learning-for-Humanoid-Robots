The goal of this project was to implement a CMAC neural network for learning a mapping between spatial visual and motor patterns.  The NAO robot performs the same simple reaching task as in the case of the MLP implementation.  In addition, the CMAC performance is compared to the MLP for this task to discuss its advantages/disadvantages given a robot scenario and environment.

Constraints:
1. Depth information in the visual space is ignored (2D coordinates in image plane)
2. The head and elbow of the NAO are held in fixed positions
3. A color marker (via a red ball) was used to collect accurate training samples

Implementations:
1. Training data was collected manually initially (75 and 150 total training samples) for comparison to the MLP implementation.  It was discovered that performance was not robust enough, so an automation training process was implemented to collect 800 total training samples to cover the whole image space.  The training_node_automated should be run in the naoqisim_ball_automated Webots world in order to collect training data.  The blob position and joint angles are stored automatically in a .dat file used for CMAC training.
2. A CMAC model was built with 2 inputs, 2 outputs, resolution of 50 and receptive field of 3 and 5.  The CMAC neural net was trained with the 75, 150, and 800 training samples each with a receptive field of 3 and then 5.  See the plots for a comparison on training times and accuracies.
3. The trained CMAC model was tested with the NAO arm tracking the red ball when manually moved.  A comparison between the CMAC and MLP was made for the training phase and execution phase.  A summary of this comparison is shown in the table below.  Overall, while the CMAC suffers from generalization issues, it is able to be trained and executed much faster than a MLP model.
