from keras.models import Sequential
from keras.layers import Dense
import numpy as np
from keras.models import model_from_json

# fix random seed for reproducibility
np.random.seed(1)

file = "sweep1"

#dataset = np.loadtxt("%s.eval.txt" % file, delimiter=" ")
#dataset = np.loadtxt("%s.txt" % file, delimiter=" ")
# split into input (X) and output (Y) variables
#X = dataset[:,0:2] # (phi, dx, dy)
#Y = dataset[:,2]   # (-np.pi/4, np.pi/4)
#Y = (Y + np.pi/4) / (np.pi/2) # (0 .. 1)

# load model
json_file = open('%s.model.json' % file, 'r')
loaded_model_json = json_file.read()
json_file.close()
model = model_from_json(loaded_model_json)
# load weights into new model
model.load_weights("%s.weights.h5" % file)
# compile model
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])

# evaluate
#results = model.evaluate(X, Y, batch_size=128)
#print('test loss, test acc:', results)

dataset = np.loadtxt("%s.xy.txt" % file, delimiter=" ")
X = dataset[:,0:2] # (phi, dx, dy)
Y = dataset[:,2]   # (-np.pi/4, np.pi/4)
Y = (Y + np.pi/4) / (np.pi/2) # (0 .. 1)

for i in range(dataset.shape[0]):
    (phi, dy) = X[i]
    best_phi = Y[i]

    inp = np.reshape([phi, dy], (1, 2))
    res = model.predict(inp)
    print("%f %f => %f (%f)" % (phi, dy, res[0][0], res[0][0]-best_phi))
