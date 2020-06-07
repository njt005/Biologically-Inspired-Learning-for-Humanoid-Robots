#! /usr/bin/python
# May 13, 2020
# Nick Tacca

from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
import os

class NeuralNet:

    def __init__(self, n_inputs, h_layer1, h_layer2, n_outputs, epochs, batch_size, lr):
        self.n_inputs = n_inputs
        self.h_layer1 = h_layer1
        self.h_layer2 = h_layer2
        self.n_outputs = n_outputs
        self.epochs = epochs
        self.batch_size = batch_size
        self.lr = lr

    def activation_function(self, x, name):
        if name == "Sigmoid":
            f = 1/(1 + np.exp(-x))
        elif name == "Tanh":
            f = (np.exp(2*x) - 1) / (np.exp(2*x) + 1)
        elif name == "ReLU":
            f = x * (x > 0)
        elif name == "Leaky_ReLU":
            f = np.where(x > 0, x, x * 0.1)
        else:
            raise ValueError('Activation function not supported')
        return f
    
    def der_activation_function(self, x, name):
        if name == "Sigmoid":
            f = 1/(1 + np.exp(-x))
            df = f*(1 - f)
        elif name == "Tanh":
            f = (np.exp(2*x) - 1) / (np.exp(2*x) + 1)
            df = 1 - f*f
        elif name == "ReLU":
            df = 1. * (x > 0)
        elif name == "Leaky_ReLU":
            df = np.where(x > 0, 1, 0.01)
        else:
            raise ValueError('Activation function not supported')
        return df
    
    def softmax(self, x):
        num = np.exp(x - np.max(x))
        den = np.sum(num)
        return num / den
    
    def feed_forward(self, X, Wh1, b1, Wh2, b2, Wo, bo):
        # Hidden layer 1
        Zh1 = np.dot(Wh1, X) + b1
        H1 = self.activation_function(Zh1, "Sigmoid")

        # Hidden layer 2
        Zh2 = np.dot(Wh2, H1) + b2
        H2 = self.activation_function(Zh2, "Sigmoid")

        # Output layer
        Zo = np.dot(Wo, H2) + bo
        output = self.softmax(Zo)
        return H1, H2, output

    def compute_loss_func(self, y_pred, y_true, name): 
        if name == "MSE":
            e = np.mean(np.power(y_true-y_pred, 2))
            # de = 2*(y_pred-y_true)/y_true.size
        elif name == "cross_entropy":
            n_samples = y_true.shape[0]
            #y_real.argmax(axis=1) returns the indices of 
            logp = -np.log(y_pred[np.arange(n_samples), y_true.argmax(axis=1)])
            #log_likelihood = -np.log(p[range(a),y_true])
            e = np.sum(logp)/n_samples
            #p[range(a),y_true] -= 1
            # de = 0
        else:
            raise ValueError('Cost function is not supported')
        return e
    
    def backpropagate(self, X, Wh1, b1, H1, Wh2, b2, H2, Wo, bo, output, y_true):

        # Derivatives
        dZo = output - y_true
        dWo = np.dot(dZo, H2.T)
        dbo = np.sum(dZo, axis=1)
        dZh2 = np.multiply(np.dot(Wo.T, dZo), self.der_activation_function(H2, "Sigmoid"))
        dWh2 = np.dot(dZh2, H1.T)
        db2 = np.sum(dZh2, axis=1)
        dZh1 = np.multiply(np.dot(Wh2.T, dZh2), self.der_activation_function(H1, "Sigmoid"))
        dWh1 = np.dot(dZh1, X.T)
        db1 = np.sum(dZh1, axis=1)
        
        # Updates
        Wh1 = Wh1 - self.lr * dWh1
        b1 = b1 - self.lr * db1
        Wh2 = Wh2 - self.lr * dWh2
        b2 = b2 - self.lr * db2
        Wo = Wo - self.lr * dWo
        bo = bo - self.lr * dbo

        return Wh1, b1, Wh2, b2, Wo, bo

def loadMNIST(prefix, folder):
    intType = np.dtype('int32').newbyteorder('>')
    nMetaDataBytes = 4 * intType.itemsize

    data = np.fromfile(folder + "/" + prefix + '-images-idx3.ubyte', dtype = 'ubyte')
    magicBytes, nImages, width, height = np.frombuffer(data[:nMetaDataBytes].tobytes(), intType)
    data = data[nMetaDataBytes:].astype(dtype = 'float32').reshape([nImages, width*height])

    labels = np.fromfile(folder + "/" + prefix + '-labels-idx1.ubyte',
                          dtype = 'ubyte')[2 * intType.itemsize:]

    return data, labels

def toHotEncoding(classification):
    hotEncoding = np.zeros([len(classification), 
                              np.max(classification) + 1])
    hotEncoding[np.arange(len(hotEncoding)), classification] = 1
    return hotEncoding
    
def main():

    # Load MNIST datasets
    dirname = os.path.dirname(os.path.abspath(__file__))
    X_train, y_train = loadMNIST("train", os.path.join(dirname, 'datasets'))
    X_test, y_test = loadMNIST("t10k", os.path.join(dirname, 'datasets'))
    y_train = toHotEncoding(y_train)
    y_test = toHotEncoding(y_test)
    
    # Normalize pixel values
    X_train /= 255
    X_test /= 255

    # Each sample in column
    X_train = X_train.T
    y_train = y_train.T
    X_test = X_test.T
    y_test = y_test.T

    # Network structure
    n_inputs = X_train.shape[0]
    h_layer1 = 350
    h_layer2 = 350
    n_outputs = y_train.shape[0]

    #Hyperparameters
    epochs = 2
    lr = 0.006
    batch_size = 80

    # Instantiate neural net class
    nn = NeuralNet(n_inputs, h_layer1, h_layer2, n_outputs, epochs, batch_size, lr)

    # Initialize weights -- bias = 0
    Wh1 = np.random.normal(-0.1, 0.1, (h_layer1, n_inputs))
    b1 = np.zeros(h_layer1)
    Wh2 = np.random.normal(-0.1, 0.1, (h_layer2, h_layer1))
    b2 = np.zeros(h_layer2)
    Wo = np.random.normal(-0.1, 0.1, (n_outputs, h_layer2))
    bo = np.zeros(n_outputs)

    # Number of training samples & batches
    m = X_train.shape[1]
    num_batches = int(m / batch_size)

    # Main training loop
    total_loss = []
    total_accuracy = []
    
    print("\nTraining...\n")

    for epoch in range(0, epochs):
        
        # Shuffle training data each epoch
        np.random.seed(138)
        shuffle_index = np.random.permutation(m)
        X_train, y_train = X_train[:,shuffle_index], y_train[:,shuffle_index]

        # Reshape data and labels for batches
        X_train = X_train.reshape((n_inputs, num_batches, batch_size))
        y_train = y_train.reshape((n_outputs, num_batches, batch_size))

        epoch_loss = []
        epoch_accuracy = []

        for iteration in range(0, num_batches):

            # Initialize training vectors
            H1 = np.zeros((h_layer1, batch_size))
            H2 = np.zeros((h_layer2, batch_size))
            output = np.zeros((n_outputs, batch_size))
            y_pred = np.zeros((n_outputs, batch_size))

            correct = 0
            for i in range(0, batch_size):
                H1[:,i], H2[:,i], output[:,i] = nn.feed_forward(X_train[:,iteration,i], Wh1, b1, Wh2, b2, Wo, bo)
                idx = np.argmax(output[:,i])
                idx_true = y_train[:,iteration,i].tolist().index(1)
                y_pred[idx,i] = 1
                if idx == idx_true:
                    correct+=1
            
            accuracy = correct / batch_size * 100

            # Compute error for batch and backpropagate
            loss = nn.compute_loss_func(output, y_train[:,iteration,:], "cross_entropy")
            Wh1, b1, Wh2, b2, Wo, bo = nn.backpropagate(X_train[:,iteration,:], Wh1, b1, H1, Wh2, b2, H2, Wo, bo, output, y_train[:,iteration,:])

            if int(iteration+1) % 1 == 0:
                print("Epoch: {}/{}, Iteration: {}/{}, Loss: {:.2f}, Accuracy: {:.2f}%".format(epoch+1,epochs,iteration+1,num_batches,loss, accuracy))

            # Append epoch loss and epoch accuracy vectors
            epoch_loss.append(loss)
            epoch_accuracy.append(accuracy)
        
        # Update Learning rate
        nn.lr = nn.lr * (1/(1+0.1*(epoch+1)))

        # Reshape data for shuffling next epoch
        X_train = X_train.reshape((n_inputs, m))
        y_train = y_train.reshape((n_outputs, m))

        # Make arrays
        epoch_loss = np.vstack(epoch_loss)
        epoch_accuracy = np.vstack(epoch_accuracy)

        # Append total loss and total accuracy vectors
        total_loss.append(epoch_loss)
        total_accuracy.append(epoch_accuracy)
    
    # Make arrays
    total_loss = np.vstack(total_loss)
    total_accuracy = np.vstack(total_accuracy)

    # Plotting loss and accuracy curves
    plt.plot(total_loss)
    plt.xlabel('Training Episodes')
    plt.ylabel('Cross Entropy Loss')
    plt.grid()
    plt.show()

    plt.plot(total_accuracy)
    plt.xlabel('Training Episodes')
    plt.ylabel('Accuracy (%)')
    plt.grid()
    plt.show()

    # Initialize vector for test set
    test_samples = y_test.shape[1]
    H1 = np.zeros((h_layer1, test_samples))
    H2 = np.zeros((h_layer2, test_samples))
    y_final = np.zeros((n_outputs, test_samples))

    print("\nTesting\n")

    correct = 0
    for j in range(0, test_samples):
        H1[:,j], H2[:,j], y_final[:,j] = nn.feed_forward(X_test[:,j], Wh1, b1, Wh2, b2, Wo, bo)
        idx = np.argmax(y_final[:,j])
        idx_true = y_test[:,j].tolist().index(1)
        y_final[idx,j] = 1
        if idx == idx_true:
            correct+=1
    
    test_accuracy = correct / test_samples * 100

    print("Accuracy: {}%".format(test_accuracy))

if __name__=='__main__':
    main()
