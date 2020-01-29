from keras.models import Sequential
from keras.layers import Dense
import numpy as np

"""

"""

file = "sweep-phi-dy--phi"

# fix random seed for reproducibility
np.random.seed(7)
dataset = np.loadtxt("%s.txt" % file, delimiter=" ")
# split into input (X) and output (Y) variables
X = dataset[:,0:2]
Y = dataset[:,2] # (-np.pi/4, np.pi/4)
Y = (Y + np.pi/4) / (np.pi/2) # (0 .. 1)
# create model
model = Sequential()
model.add(Dense(8, input_dim=2, activation='relu'))
model.add(Dense(1, activation='relu'))
# save the model
model_json = model.to_json()
with open("%s.model.json" % file, "w") as json_file:
    json_file.write(model_json)
# Compile model
model.compile(loss='mean_squared_error', optimizer='adam')
# Fit the model
model.fit(X, Y, validation_split=0.33, epochs=50, batch_size=16)
model.save_weights("%s.weights.h5" % file)
