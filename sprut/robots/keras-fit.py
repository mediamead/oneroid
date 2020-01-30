from keras.models import Sequential
from keras.layers import Dense
import numpy as np

import sys
dfile = sys.argv[1]
mfile = sys.argv[2]

# fix random seed for reproducibility
np.random.seed(7)
dataset = np.loadtxt(dfile, delimiter=" ")
# split into input (X) and output (Y) variables
X = dataset[:,0:2]
Y = dataset[:,2] # (-np.pi/4, np.pi/4)
Y = (Y + np.pi/4) / (np.pi/2) # (0 .. 1)
# create model
model = Sequential()
model.add(Dense(32, input_dim=2, activation='relu'))
model.add(Dense(1))
# Compile model
model.compile(loss='mean_squared_error', optimizer='adam')

# Fit the model
model.fit(X, Y, validation_split=0.33, epochs=25, batch_size=16)

# save the model
model_json = model.to_json()
with open("%s.model.json" % mfile, "w") as json_file:
    json_file.write(model_json)
model.save_weights("%s.weights.h5" % mfile)

for i in range(dataset.shape[0]):
    (phi, dy) = X[i]
    best_phi = Y[i]

    inp = np.reshape([phi, dy], (1, 2))
    res = model.predict(inp)
    print("%f %f => %f (%f)" % (phi, dy, res[0][0], best_phi))
