The goal of this project was to implement a feed forward neural network from scratch using numpy to learn a mapping between spatial visual and motor patterns.  The mapping between visual and motor patterns enables the NAO robot to perform a simple reaching and following task.

Constraints:
1. Depth information in the visual space is ignored (2D coordinates in image plane)
2. The head and elbow of the NAO are held in fixed positions
3. A color marker (via a red ball) was used to collect accurate training samples

Implementations:
MNIST
1. A 4 layer neural network was implemented using only numpy to classify the MNIST dataset.  Results indicate ~90% accuracy.  Further tuning could be implemented to achieve a higher accuracy.  However, the current simple MLP is sufficient for the following NAO ball track task below.

NAO Ball Track
1. Training data was collected for the feed forward neural network (visual blob position as input, shoulder position as output).  By running the training node in the naoqisim_ball.wbt world, the NAO adjusts its arm position covering the range of the input visual space.  At each position, the red ball is manually moved and its centroid blob position is printed in the terminal for data collection.  Alternatively, automated training can be used to collect data (this was implemented at a later date during the CMAC project since it required many more training samples to work well).  To conduct automated training, run the training_node_automated in the naoqisim_ball_automated.wbt (see CMAC folder) and each blob and arm position will automatically be recorded in a .dat file.
2. The training data was then normalized between 0 and 1 and fed through an adapted version of the MNIST neural network for training.  This neural network output continuous values between 0 and 1 which were then re-normalized as joint-angle positions for the NAO to move its arm in order to track the ball.
3. Following the training, the feed forward network allows the NAO to track the ball using its arm when you move the red ball manually.
