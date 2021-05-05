import numpy as np
import random
from math import sqrt, pi, exp

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories
    def __init__(self):
        self.classes = ['left', 'keep', 'right']

    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, vars):
        # The following implementation simply extracts the four raw values
        # given by the input data, i.e. s, d, s_dot, and d_dot.
        s, d, s_dot, d_dot = vars
        return s, d, s_dot, d_dot

    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").
    def train(self, X, Y):
        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''
        # TODO: implement code.

        data_seperation = {}
        self.data_stat = {}

        for i, clss in enumerate(Y) : 
            if clss not in data_seperation : 
                data_seperation[clss] = []
                self.data_stat[clss] = {}

            data_seperation[clss].append(X[i])
        
        for clss in data_seperation : 
            data = np.array(data_seperation[clss])

            means = np.mean(data, axis=0)
            stds = np.std(data, axis=0)

            self.data_stat[clss]['means'] = means
            self.data_stat[clss]['stds'] = stds
            self.data_stat[clss]['len'] = data.shape[0]
        
        

    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
        # TODO: implement code.
        probs = []
        total_len = sum(self.data_stat[clss]['len'] for clss in self.data_stat)

        for clss in self.data_stat : 
            prob = self.data_stat[clss]['len'] / total_len

            for i in range(len(observation)) : 
                mean = self.data_stat[clss]['means'][i]
                std = self.data_stat[clss]['stds'][i]
                prob *= gaussian_prob(observation[i], mean, std)

            probs.append(prob)

        return list(self.data_stat.keys())[np.argmax(probs)]

